import socket
import json
import threading
import queue          # ← non-blocking state send queue
import numpy as np
import omni.kit.app
import omni.ui as ui
import time
import gc
import struct
import io
import traceback
from omni.isaac.core.articulations import Articulation

# ── Viewport / camera capture ─────────────────────────────────────────────────
# Isaac Sim exposes several APIs depending on version.  We try them in order
# and fall back gracefully so the robot control still works if none is found.

# Method A: omni.kit.viewport.utility  (Isaac Sim 2022.2+)
try:
    import omni.kit.viewport.utility as _vp_util
    _HAS_VP_UTIL = True
except ImportError:
    _HAS_VP_UTIL = False

# Method B: omni.replicator / omni.syntheticdata  (Isaac Sim 2023+)
try:
    import omni.replicator.core as rep
    _HAS_REP = True
except ImportError:
    _HAS_REP = False

# PIL for JPEG encoding
try:
    from PIL import Image as _PILImage
    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False

# ==============================================================================
# 1. GLOBAL CLEANUP (Prevents "Port In Use" errors)
# ==============================================================================
if "tcp_server" in globals():
    print("♻️ Found existing server. Cleaning up...")
    try: globals()["tcp_server"].stop()
    except: pass
    del globals()["tcp_server"]
    # Give daemon threads time to exit after their sockets were closed
    time.sleep(0.5)

# Force close any lingering sockets in memory
for obj in gc.get_objects():
    if isinstance(obj, socket.socket):
        try: obj.close()
        except: pass

# ==============================================================================
# 2. DUAL TCP SERVER CLASS (With UI + Sliders)
# ==============================================================================
class UR3DualTCPServer:
    def __init__(self):
        # --- Configuration ---
        self.listen_ip = "0.0.0.0"
        self.cmd_port   = 11020  # Port to Receive Commands
        self.state_port = 11021  # Port to Send State
        self.video_port = 11022  # Port to Stream Video (JPEG frames)
        
        self.ur3_path = "/ur3"   # <--- CHECK THIS MATCHES YOUR STAGE
        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]

        # --- State ---
        self.running = True
        self.cmd_conn   = None
        self.state_conn = None
        self.video_conn = None          # video client socket
        self.video_fps  = 30            # target frame-rate for video stream
        self.num_dof = 6
        self.target_q = np.zeros(self.num_dof, dtype=np.float64)
        self.frame_count = 0
        self._pending_ts = None         # holds {t1, cmd_seq, t3} from latest command
        # Camera capture state (lazy-initialised by _rep_get_frame)
        self._rep_annotator  = None
        self._rep_render_prod = None
        # If replicator or syntheticdata raises async/coroutine errors at runtime
        # we mark it as broken and stop calling it to prevent log spam.
        self._rep_broken = False
        # Server-side listen sockets — kept so stop() can close them immediately
        self._cmd_sock   = None
        self._state_sock = None
        self._video_sock = None
        # Non-blocking outbound state queue.
        # _on_update puts serialised payloads here (never blocks the physics thread).
        # A dedicated sender thread drains the queue via sendall.
        # maxsize=4 means if Unity is far behind we drop old state rather than
        # letting the queue grow unbounded and wasting memory.
        self._state_send_q: queue.Queue[bytes] = queue.Queue(maxsize=4)
        # Latest JPEG frame produced on the MAIN THREAD (_on_update) and
        # consumed by the video send thread.  Lock protects the swap.
        self._latest_jpeg      = None   # bytes | None
        self._latest_jpeg_lock = threading.Lock()
        self._video_frame_ready = threading.Event()  # set each time a new frame arrives
        self._target_q_lock = threading.Lock()        # protects target_q across cmd/physics/UI threads
        self._pending_ts_lock = threading.Lock()       # atomic swap for _pending_ts

        # --- Robot Setup ---
        try:
            self.ur3 = Articulation(self.ur3_path)
            self.ur3.initialize()
            print(f"✅ Robot found at: {self.ur3_path}")
        except:
            print(f"⚠️ Robot not found at {self.ur3_path}. Check Prim Path!")
            self.ur3 = None

        # --- UI Setup ---
        self.slider_models = []
        self._build_ui()

        # --- Start Networking Threads ---
        threading.Thread(target=self.listen_cmd,   daemon=True).start()
        threading.Thread(target=self.listen_state, daemon=True).start()
        threading.Thread(target=self.listen_video, daemon=True).start()
        if not _HAS_PIL:
            print("⚠️ [video] Pillow not installed — video streaming disabled. Run: pip install Pillow")
        print(f"🚀 Dual TCP Server Active! (Cmd:{self.cmd_port}, State:{self.state_port}, Video:{self.video_port})")

        # --- Start Physics Loop ---
        self.sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_update)

    # --------------------------------------------------------------------------
    # THREAD 1: RECEIVE COMMANDS (Port 11020)
    # --------------------------------------------------------------------------
    def listen_cmd(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._cmd_sock = sock
        try:
            sock.bind((self.listen_ip, self.cmd_port))
            sock.listen(1)
            
            while self.running:
                try:
                    client, addr = sock.accept()
                except Exception:
                    break  # sock closed by stop()
                print(f"📥 Cmd client connected: {addr}")
                self.cmd_conn = client
                
                buffer = ""
                try:
                    while self.running:
                        try:
                            chunk = client.recv(1024)
                        except BlockingIOError:
                            continue  # no data yet on non-blocking socket — keep looping
                        except (ConnectionResetError, BrokenPipeError, OSError):
                            print("📥 Cmd client connection lost.")
                            break
                        if not chunk:
                            print("📥 Cmd client closed connection (EOF).")
                            break
                        buffer += chunk.decode('utf-8', errors='replace')
                        while "\n" in buffer:
                            line, buffer = buffer.split("\n", 1)
                            line = line.strip()
                            if line:
                                try:
                                    self.process_msg(line)
                                except Exception as pe:
                                    print(f"📥 process_msg error: {pe}")
                except Exception as e:
                    print(f"📥 Cmd recv error: {type(e).__name__}: {e}")
                finally:
                    self.cmd_conn = None
                    try: client.close()
                    except: pass
                    print("📥 Cmd client disconnected.")
        except Exception as e:
            if self.running:
                print(f"📥 Cmd socket error: {type(e).__name__}: {e}")
        finally: sock.close()

    # --------------------------------------------------------------------------
    # THREAD 2: SEND STATE (Port 11021)
    # --------------------------------------------------------------------------
    def listen_state(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._state_sock = sock
        try:
            sock.bind((self.listen_ip, self.state_port))
            sock.listen(1)

            while self.running:
                try:
                    client, addr = sock.accept()
                except Exception:
                    break  # sock closed by stop()
                print(f"📡 State client connected: {addr}")

                # Drain stale payloads BEFORE exposing the new client to _on_update.
                # If done after setting state_conn, _on_update could enqueue a fresh
                # payload that then gets drained here, losing the first state update.
                while not self._state_send_q.empty():
                    try: self._state_send_q.get_nowait()
                    except queue.Empty: break

                self.state_conn = client

                # ── Per-session alive event ───────────────────────────────────
                # A NEW event is created for every client session so a dying
                # sender thread from a previous session can never clear the
                # current session's event and cause a false disconnect.
                session_alive_evt = threading.Event()
                session_alive_evt.set()

                # ── Dedicated sender thread ───────────────────────────────────
                # All sendall() calls happen here, NOT on the physics thread.
                # If sendall blocks (Unity slow), only this thread waits —
                # the physics thread keeps running at full speed.
                def _state_sender(conn, alive_evt):
                    try:
                        while self.running and self.state_conn is conn:
                            try:
                                payload = self._state_send_q.get(timeout=0.5)
                            except queue.Empty:
                                continue
                            try:
                                conn.sendall(payload)   # may block here — that's OK
                            except (BrokenPipeError, ConnectionResetError,
                                    OSError) as e:
                                print(f"📡 [state_sender] connection lost — {type(e).__name__}: {e}")
                                alive_evt.clear()  # only clears THIS session's event
                                break
                            except Exception as e:
                                # Transient / unexpected errors: log but keep running
                                print(f"📡 [state_sender] sendall error (retrying) — {type(e).__name__}: {e}")
                    finally:
                        pass  # listen_state's finally block handles cleanup

                sender = threading.Thread(
                    target=_state_sender, args=(client, session_alive_evt), daemon=True)
                sender.start()

                # Enable TCP keepalive on the state socket so the OS detects
                # a silently-dead Unity client and raises an error in sendall,
                # rather than blocking forever or never disconnecting.
                try:
                    client.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                except Exception:
                    pass  # not supported on all platforms — safe to ignore

                try:
                    while self.running:
                        if not session_alive_evt.is_set():
                            print("📡 State: connection lost — closing client.")
                            break
                        time.sleep(0.2)
                except Exception as e:
                    print(f"📡 State poll loop exception: {type(e).__name__}: {e}")
                finally:
                    print("📡 State client disconnected.")
                    session_alive_evt.clear()
                    self.state_conn = None
                    try: client.close()
                    except: pass
                    sender.join(timeout=1.0)
        except Exception as e:
            if self.running:
                print(f"📡 State socket error: {type(e).__name__}: {e}")
        finally: sock.close()

    # --------------------------------------------------------------------------
    # THREAD 3: STREAM VIDEO (Port 11022)
    # Each frame is sent as:
    #   [4 bytes big-endian length][JPEG bytes]
    # Unity reads the length header first, then reads exactly that many bytes.
    # --------------------------------------------------------------------------
    def listen_video(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._video_sock = sock
        try:
            sock.bind((self.listen_ip, self.video_port))
            sock.listen(1)
            print(f"📷 Video server listening on port {self.video_port}")

            while self.running:
                try:
                    client, addr = sock.accept()
                except Exception:
                    break  # sock was closed by stop() — exit cleanly
                print(f"📷 Video client connected: {addr}")
                self.video_conn = client
                try:
                    self._send_video_loop(client)
                finally:
                    self.video_conn = None
                    try: client.close()
                    except: pass
                    print("📷 Video client disconnected.")
        except Exception as e:
            if self.running:
                print(f"📷 Video socket error: {e}")
        finally:
            sock.close()

    def _send_video_loop(self, client):
        """Send pre-captured JPEG frames (produced by _on_update on main thread).

        TCP back-pressure note:
        - sendall() can block if Unity's TCP receive buffer is full (slow reader).
        - We tolerate that here — the video thread is the only one that blocks.
        - The main/physics thread is never touched: it only swaps _latest_jpeg
          under the lock and sets the event.
        - We snapshot the bytes under the lock then release before sendall so the
          main thread is never held waiting for a slow network write.
        - Two separate sendall() calls (header then body) avoid allocating a
          large concatenated bytes object every frame.
        """
        while self.running and self.video_conn is client:
            # Block until a new frame is ready (or timeout so we check running/conn)
            self._video_frame_ready.wait(timeout=0.1)
            self._video_frame_ready.clear()

            # Snapshot under lock — release immediately before network I/O
            with self._latest_jpeg_lock:
                jpeg_bytes = self._latest_jpeg
                self._latest_jpeg = None      # consume so we don't re-send stale frame

            if jpeg_bytes is None:
                continue
            try:
                header = struct.pack(">I", len(jpeg_bytes))
                client.sendall(header)         # two calls avoids one large allocation
                client.sendall(jpeg_bytes)
            except Exception as e:
                print(f"📷 Video send error: {e}")
                break
        try:
            client.close()
        except:
            pass

    def _capture_viewport_jpeg(self):
        """
        Grab the active Isaac Sim viewport and return JPEG bytes.

        Method order:
          1. omni.replicator.core annotator  — best quality, Isaac Sim 2023+
          2. omni.kit.viewport.utility       — fallback (older builds)
          3. Solid-colour test frame         — always works, proves Unity pipeline
        """
        if not _HAS_PIL:
            return None   # error printed once at startup if missing

        # ── Method 1: replicator annotator (preferred) ────────────────────────
        if _HAS_REP:
            try:
                rgba = self._rep_get_frame()
                if rgba is not None:
                    return self._rgba_to_jpeg(rgba)
            except Exception as e:
                print(f"📷 [rep] {type(e).__name__}: {e}")

        # ── Method 2: viewport utility (Isaac Sim 2022.x sync API) ───────────
        if _HAS_VP_UTIL:
            try:
                vp = _vp_util.get_active_viewport()
                if vp is not None:
                    # capture_viewport_to_buffer takes NO arguments in Isaac Sim 2023+
                    # (the viewport is already set as active)
                    result = _vp_util.capture_viewport_to_buffer()
                    if result is not None:
                        arr = None
                        if isinstance(result, np.ndarray):
                            arr = result.astype(np.uint8)
                        elif hasattr(result, '__array_interface__'):
                            arr = np.array(result, dtype=np.uint8)
                        elif isinstance(result, dict) and 'data' in result:
                            h = result.get('height', 480)
                            w = result.get('width', 640)
                            arr = np.frombuffer(result['data'], dtype=np.uint8).reshape(h, w, 4)
                        if arr is not None:
                            return self._rgba_to_jpeg(arr)
            except Exception as e:
                print(f"📷 [vp_util] {type(e).__name__}: {e}")

        # ── Method 3: solid-colour test frame (always works) ─────────────────
        # Shows alternating blue/red every second. Remove once real capture works.
        try:
            colour = (0, 80, 200) if int(time.time()) % 2 == 0 else (200, 50, 0)
            test_arr = np.full((480, 640, 3), colour, dtype=np.uint8)
            img = _PILImage.fromarray(test_arr, mode="RGB")
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=50)
            return buf.getvalue()
        except Exception as e:
            print(f"📷 [test_frame] {e}")

        return None

    # ── Replicator annotator helpers ──────────────────────────────────────────
    def _rep_get_frame(self):
        """
        Use omni.replicator to grab an RGB frame from the first Camera prim.
        Isaac Sim 2023+: get_data() returns either:
          - a raw numpy array  (shape H×W×4, dtype uint8)
          - a dict with 'data', 'height', 'width' keys  (older builds)
        """
        # If replicator was detected to be broken at runtime, skip it.
        if getattr(self, '_rep_broken', False):
            return None

        if self._rep_annotator is None:
            cam_path = self._find_camera_prim()
            if cam_path is None:
                print("📷 [rep] No Camera prim found in USD stage")
                return None
            print(f"📷 [rep] Creating render product for: {cam_path}")
            try:
                render_prod = rep.create.render_product(cam_path, (640, 480))
                self._rep_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
                self._rep_annotator.attach([render_prod])
                self._rep_render_prod = render_prod
            except Exception as e:
                # Some versions of omni.syntheticdata / replicator may trigger
                # runtime coroutine warnings or other TF_PYTHON_EXCEPTIONs when
                # used in this context. Mark replicator as unusable so we
                # don't repeatedly call it and spam the logs; fall back to
                # viewport utility or test frame instead.
                print(f"📷 [rep] Initialization FAILED — disabling replicator: {type(e).__name__}: {e}")
                traceback.print_exc()
                self._rep_broken = True
                return None

        try:
            data = self._rep_annotator.get_data()
        except Exception as e:
            # Runtime errors (including coroutine warnings from syntheticdata)
            # can surface here. Disable replicator to stop log flooding and
            # fall back to other capture methods.
            print(f"📷 [rep] get_data() FAILED — disabling replicator: {type(e).__name__}: {e}")
            traceback.print_exc()
            self._rep_broken = True
            return None

        if data is None:
            return None

        # Isaac Sim 2023+: data is a numpy array directly
        if isinstance(data, np.ndarray):
            if data.size == 0:
                return None
            # Flatten to 1-D then reshape to H×W×C
            flat = data.flatten()
            pixels = flat.size // 4
            if pixels == 0:
                return None
            # Try to use annotator info for shape; fall back to 640×480
            try:
                info = self._rep_annotator.get_info()
                h = info.get('height', 480)
                w = info.get('width', 640)
            except Exception:
                h, w = 480, 640
            return np.array(flat, dtype=np.uint8).reshape(h, w, 4)

        # Older builds: data is a dict
        if isinstance(data, dict):
            arr = data.get("data", None)
            if arr is None:
                return None
            h = data.get("height", 480)
            w = data.get("width",  640)
            return np.array(arr, dtype=np.uint8).reshape(h, w, 4)

        return None

    # ── USD stage camera scan ─────────────────────────────────────────────────
    def _find_camera_prim(self):
        """
        Walk the USD stage and return the path of the first Camera prim found.
        Prefers a prim whose name contains 'Camera' or 'camera'.
        Returns None if no camera exists.
        """
        try:
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            if stage is None:
                return None
            from pxr import UsdGeom
            cameras = []
            for prim in stage.Traverse():
                if prim.IsA(UsdGeom.Camera):
                    cameras.append(str(prim.GetPath()))
            if not cameras:
                return None
            # Prefer a prim with "Camera" in the name
            named = [p for p in cameras if "camera" in p.lower()]
            return named[0] if named else cameras[0]
        except Exception as e:
            print(f"📷 [find_cam] {e}")
            return None

    def _usd_camera_grab(self):
        # Kept for reference — not called by _capture_viewport_jpeg anymore.
        # If replicator fails, check _find_camera_prim() returns a valid path.
        pass

    # ── JPEG encoder ──────────────────────────────────────────────────────────
    @staticmethod
    def _rgba_to_jpeg(rgba_array, quality=75):
        """Convert an (H, W, 4) or (H, W, 3) uint8 numpy array to JPEG bytes."""
        arr = np.asarray(rgba_array, dtype=np.uint8)
        if arr.ndim == 3 and arr.shape[2] == 4:
            arr = arr[:, :, :3]   # drop alpha
        img = _PILImage.fromarray(arr, mode="RGB")
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=quality)
        return buf.getvalue()
    # --------------------------------------------------------------------------
    def process_msg(self, json_str):
        try:
            msg = json.loads(json_str)

            # T3 = Isaac receive time (ns, monotonic) — stamp immediately
            t3 = time.monotonic_ns()

            if "joints" in msg:
                with self._target_q_lock:
                    self.target_q[:] = np.array(msg["joints"], dtype=np.float64)
            elif "delta" in msg:
                idx = int(msg["delta"][0])
                val = float(msg["delta"][1])
                with self._target_q_lock:
                    self.target_q[idx] += val
            else:
                return  # nothing to track timing for

            # Extract Unity-side timestamps for echo back to Unity
            # t0: Unity input-capture time (UTC ms) — used for MTP = T7 - t0_echo
            # t1: Unity send time (UTC ms)           — used for RTT = T6 - t1_echo
            # cmd_seq: per-command counter            — used as pairing guard in LatencyLogger
            t0      = int(msg.get("t0", 0))
            t1      = int(msg.get("t1", 0))
            cmd_seq = int(msg.get("cmd_seq", 0))
            with self._pending_ts_lock:
                self._pending_ts = {"t0": t0, "t1": t1, "cmd_seq": cmd_seq, "t3": t3}
        except:
            pass

    # --------------------------------------------------------------------------
    # MAIN LOOP: PHYSICS & UI UPDATE
    # --------------------------------------------------------------------------
    _on_update_send_err_time = 0.0   # class-level rate-limit for error prints
    _last_frame_time         = 0.0   # throttle video capture to video_fps

    def _on_update(self, dt):
        if not self.running: return

        # T4 = physics applied (ns, monotonic)
        t4 = time.monotonic_ns()

        # 1. Apply Physics
        if self.ur3:
            with self._target_q_lock:
                q_snapshot = self.target_q.copy()
            self.ur3.set_joint_positions(q_snapshot)

        # 2. Sync UI Sliders
        with self._target_q_lock:
            q_snapshot = self.target_q.copy()
        for i, model in enumerate(self.slider_models):
            if abs(model.as_float - q_snapshot[i]) > 0.001:
                model.set_value(float(q_snapshot[i]))

        # 3. Send State (If Client Connected)
        if self.state_conn:
            try:
                if self.ur3:
                    try:
                        actual_pos = self.ur3.get_joint_positions()
                        safe_pos = [0.0 if np.isnan(x) else float(x) for x in actual_pos]
                    except:
                        with self._target_q_lock:
                            safe_pos = [0.0 if np.isnan(x) else float(x) for x in self.target_q.copy()]
                else:
                    with self._target_q_lock:
                        safe_pos = [0.0 if np.isnan(x) else float(x) for x in self.target_q.copy()]

                t5 = time.monotonic_ns()   # T5: Isaac sendto() timestamp — ns, same clock as T3/T4
                # Atomically take the pending timestamps — swap with None under lock
                with self._pending_ts_lock:
                    ts = self._pending_ts or {}
                    self._pending_ts = None

                self.frame_count += 1

                # All Isaac timestamps sent as nanoseconds (time.monotonic_ns()).
                # Unity UTC ms timestamps (t0_echo, t1_echo) are echoed back unchanged.
                # cmd_seq_echo lets LatencyLogger pair each response to its exact command.
                payload = (json.dumps({
                    "seq":          self.frame_count,
                    "cmd_seq_echo": ts.get("cmd_seq", 0),   # echoed Unity cmd_seq — pairing key
                    "t0_echo":      ts.get("t0", 0),        # echoed Unity T0 UTC ms — for MTP
                    "t1_echo":      ts.get("t1", 0),        # echoed Unity T1 UTC ms — for RTT
                    "t3_recv":      ts.get("t3", 0),        # T3: Isaac recvfrom() ns
                    "t4_physics":   t4,                     # T4: Isaac physics applied ns
                    "t5_send":      t5,                     # T5: Isaac sendto() ns
                    "state":        safe_pos,
                }) + "\n").encode('utf-8')

                # ── Non-blocking put ──────────────────────────────────────────
                # If the queue is full (Unity is not keeping up) drop the oldest
                # packet so the physics thread is NEVER blocked by TCP back-pressure.
                try:
                    self._state_send_q.put_nowait(payload)
                except queue.Full:
                    try: self._state_send_q.get_nowait()   # discard oldest
                    except queue.Empty: pass
                    try: self._state_send_q.put_nowait(payload)
                    except queue.Full: pass                 # give up gracefully
            except Exception as e:
                now = time.time()
                if now - UR3DualTCPServer._on_update_send_err_time >= 1.0:
                    UR3DualTCPServer._on_update_send_err_time = now
                    print(f"📡 [_on_update] state prep FAILED — {type(e).__name__}: {e}")
                    traceback.print_exc()

        # 4. Capture video frame (MAIN THREAD ONLY — Isaac Sim APIs are not thread-safe)
        #    Only capture when a video client is connected, throttled to video_fps.
        if self.video_conn is not None and _HAS_PIL:
            now = time.time()
            interval = 1.0 / max(1, self.video_fps)
            if now - UR3DualTCPServer._last_frame_time >= interval:
                UR3DualTCPServer._last_frame_time = now
                try:
                    jpeg = self._capture_viewport_jpeg()
                    if jpeg is not None:
                        with self._latest_jpeg_lock:
                            self._latest_jpeg = jpeg
                        self._video_frame_ready.set()
                except Exception as e:
                    print(f"📷 [capture] {type(e).__name__}: {e}")

    # --------------------------------------------------------------------------
    # UI BUILDER
    # --------------------------------------------------------------------------
    def _build_ui(self):
        self.window = ui.Window("TCP UR3 Control", width=350, height=450)
        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=8, style={"margin": 10}):
                    ui.Label("TCP SERVER RUNNING", style={"color": 0xFF00FF00, "font_size": 20})
                    
                    ui.Spacer(height=10)

                    for i, name in enumerate(self.joint_names):
                        with ui.HStack(height=30):
                            ui.Label(name.replace("_joint", ""), width=120)
                            slider = ui.FloatSlider(min=-3.14, max=3.14)
                            slider.model.set_value(float(self.target_q[i]))
                            self.slider_models.append(slider.model)
                            
                            def make_cb(idx):
                                def _cb(m):
                                    with self._target_q_lock:
                                        self.target_q[idx] = m.as_float
                                return _cb
                            slider.model.add_value_changed_fn(make_cb(i))

                    ui.Spacer(height=20)
                    ui.Button("STOP SERVER", height=40, clicked_fn=self.stop, style={"background_color": 0xFF5555AA})

    def stop(self):
        self.running = False

        # Cancel the physics update subscription properly.
        # Setting self.sub = None alone does NOT unsubscribe in Isaac Sim.
        if self.sub is not None:
            try: self.sub.unsubscribe()
            except: pass
            self.sub = None

        # Close server listen sockets — unblocks accept() in all daemon threads
        # and releases the OS port bindings immediately.
        for s in (self._cmd_sock, self._state_sock, self._video_sock):
            if s is not None:
                try: s.close()
                except: pass
        self._cmd_sock = self._state_sock = self._video_sock = None

        # Close active client connections
        for conn in (self.cmd_conn, self.state_conn, self.video_conn):
            if conn is not None:
                try: conn.close()
                except: pass
        self.cmd_conn = self.state_conn = self.video_conn = None

        # Clear video frame slot
        with self._latest_jpeg_lock:
            self._latest_jpeg = None
        self._video_frame_ready.set()  # unblock send thread so it can exit

        # Flush the state send queue so the sender thread's get() returns quickly
        while not self._state_send_q.empty():
            try: self._state_send_q.get_nowait()
            except queue.Empty: break

        # Clean up replicator annotator
        if self._rep_annotator is not None:
            try: self._rep_annotator.detach()
            except: pass
            self._rep_annotator = None
        if self._rep_render_prod is not None:
            try: self._rep_render_prod.destroy()
            except: pass
            self._rep_render_prod = None

        if self.window:
            self.window.visible = False
            self.window = None

        print("✅ Server Stopped.")

# ==============================================================================
# START
# ==============================================================================
globals()["tcp_server"] = UR3DualTCPServer()

