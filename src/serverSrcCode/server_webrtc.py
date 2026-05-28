"""
WebRTC-enabled Isaac Sim UR3 Server
=====================================
Drop-in replacement for server.py.

Instead of raw UDP, this version:
  • Opens a WebSocket to the signaling server
  • Negotiates a WebRTC peer connection with the Unity client
  • Streams joint state over a reliable/ordered DataChannel ("robot_state")
  • Receives joystick commands over another DataChannel ("commands")

Dependencies (install in Isaac Sim's Python or in a venv):
    pip install aiortc websockets aiohttp

Because Isaac Sim's update loop is synchronous (omni.kit), we run the
asyncio event-loop on a background thread and bridge with thread-safe queues.
"""

import json
import time
import threading
import asyncio
import queue
import numpy as np
import csv
import os
import subprocess
from datetime import datetime, timezone

# Isaac Sim imports
import omni.kit.app
import omni.usd
import omni.ui as ui
from omni.isaac.core.articulations import Articulation
from pxr import PhysxSchema, UsdPhysics

# WebRTC / signaling
try:
    from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer
    import websockets
except ImportError as e:
    print(f"❌  Missing WebRTC dependencies. Run:  pip install aiortc websockets")
    raise

# ==============================================================================
# CHRONY CLOCK WRAPPER (same as UDP server)
# ==============================================================================
class ChronyClock:
    """
    Thin clock wrapper that relies on Linux chrony/chronyd to discipline the
    system clock. No per-process NTP thread is used.
    """
    def __init__(self, check_sync=True):
        self._check_sync = check_sync
        self._sync_ok = True

    def _probe_chrony_sync(self):
        """Best-effort chrony sync probe via `chronyc tracking`."""
        try:
            result = subprocess.run(
                ["chronyc", "tracking"],
                capture_output=True,
                text=True,
                timeout=1.0,
                check=False,
            )
            if result.returncode != 0:
                return False

            for line in result.stdout.splitlines():
                if "Leap status" in line:
                    return "Normal" in line
            return False
        except Exception:
            return False

    def start(self):
        """Chrony-backed mode uses system UTC clock; no background thread."""
        if self._check_sync:
            self._sync_ok = self._probe_chrony_sync()
            if self._sync_ok:
                print("[ChronyClock] chrony reports synchronized (Leap status: Normal)")
            else:
                print("[ChronyClock] WARNING: chrony status unknown or not synced yet; using system UTC time")
        else:
            self._sync_ok = True
            print("[ChronyClock] Sync probe disabled; using system UTC time")

    def stop(self):
        """No-op for API compatibility."""
        print("[ChronyClock] Stopped.")

    def get_corrected_time_ms(self):
        """Get current system UTC time in milliseconds (chrony-disciplined)."""
        return int(time.time() * 1000)

    def get_offset_ms(self):
        """Not tracked in-process in chrony mode."""
        return 0.0

    def is_synced(self):
        """Best-effort chrony sync status from startup probe."""
        return self._sync_ok

# ==============================================================================
# 1. GLOBAL CLEANUP (Same pattern as original)
# ==============================================================================
if "isaac_webrtc_server" in globals():
    print("♻️ Found existing server. Cleaning up...")
    globals()["isaac_webrtc_server"].stop()
    del globals()["isaac_webrtc_server"]

# ==============================================================================
# 2. CONFIGURATION
# ==============================================================================
SIGNALING_URL = "ws://10.9.71.137:8765"     # <-- Point to your signaling server
STUN_SERVER   = "stun:stun.l.google.com:19302"

# ==============================================================================
# 3. ASYNC WebRTC BRIDGE (runs on a background thread)
# ==============================================================================
class WebRTCBridge:
    """
    Runs an asyncio event-loop on a daemon thread.
    Provides thread-safe queues for the Isaac Sim main thread to:
      • push outgoing state  (state_out_queue)
      • pull incoming cmds   (cmd_in_queue)
    """

    def __init__(self, signaling_url: str, clock: ChronyClock = None):
        self.signaling_url = signaling_url
        # Increased maxsize to 60 to handle 60Hz updates without dropping too many frames
        self.state_out_queue: queue.Queue = queue.Queue(maxsize=60)  # drop-old policy
        # Application-level command queue (DataChannel callback -> main thread)
        self.cmd_in_queue: queue.Queue    = queue.Queue(maxsize=64)
        self.connected = threading.Event()
        self._loop: asyncio.AbstractEventLoop = None
        self._thread: threading.Thread = None
        self._running = True
        self._dc_state = None        # DataChannel for state  (we create)
        self._dc_commands = None     # DataChannel for cmds   (we create)
        self._pc: RTCPeerConnection = None
        self.clock = clock

        # --- Per-command timestamps: each command carries its own t0/t1/t3 ---
        # These are set per-frame in _on_update from the commands actually
        # processed in that frame, NOT from a global that can be overwritten.
        # _frame_t0/t1/t3/_frame_tq hold the timestamps of the most recent command
        # processed in the CURRENT physics frame.
        self._frame_t0 = 0
        self._frame_t1 = 0
        self._frame_t3 = 0
        self._frame_tq = 0
        self._frame_cmd_seq = -1

        # --- Queue size accounting for UDP-style logging ---
        self._cmd_queue_bytes = 0
        self._cmd_queue_lock = threading.Lock()
        self.max_queue_size_bytes = 10 * 1024 * 1024

    # ------------------------------------------------------------------
    def start(self):
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def _run_loop(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._main())

    # ------------------------------------------------------------------
    async def _main(self):
        while self._running:
            try:
                await self._session()
            except Exception as e:
                print(f"[WebRTC] Session error: {e}  — retrying in 3 s")
            if self._running:
                await asyncio.sleep(3)

    async def _session(self):
        config = RTCConfiguration(iceServers=[RTCIceServer(urls=[STUN_SERVER])])
        self._pc = RTCPeerConnection(configuration=config)

        # ---------- Detect peer disconnect at ICE level ----------
        @self._pc.on("connectionstatechange")
        async def on_conn_state_change():
            state = self._pc.connectionState
            print(f"[WebRTC] 🔗 Connection state: {state}")
            if state in ("failed", "closed", "disconnected"):
                print(f"[WebRTC] 🔌 Peer connection {state} — clearing connected flag")
                self.connected.clear()

        # ---------- Create Data Channels ----------
        # Server creates 'robot_state' (Send-only from Isaac to Unity)
        self._dc_state = self._pc.createDataChannel("robot_state", ordered=False, maxRetransmits=0)

        @self._dc_state.on("open")
        def on_state_open():
            print("[WebRTC] ✅ DataChannel 'robot_state' OPEN (Send-only)")
            # We don't set connected here anymore, we wait for the signaling to finish

        @self._dc_state.on("close")
        def on_state_close():
            print("[WebRTC] 🔌 DataChannel 'robot_state' CLOSED")
            self.connected.clear()

        # Handle data channels created by the remote peer (Unity)
        # Unity will create 'commands' (Receive-only for Isaac)
        @self._pc.on("datachannel")
        def on_datachannel(channel):
            print(f"[WebRTC] 📥 Remote DataChannel: {channel.label}")
            if channel.label == "commands":
                self._dc_commands = channel
                
                @channel.on("open")
                def on_cmd_open():
                    print("[WebRTC] ✅ DataChannel 'commands' OPEN (Receive-only)")

                @channel.on("message")
                def on_msg(msg):
                    try:
                        if self.clock is not None:
                            t3 = self.clock.get_corrected_time_ms()
                        else:
                            t3 = int(time.time() * 1000)  # T3 = Isaac receive time (UTC ms)
                        raw_msg = msg if isinstance(msg, str) else msg.decode("utf-8")
                        size_bytes = len(raw_msg.encode("utf-8"))

                        try:
                            parsed = json.loads(raw_msg)
                        except Exception:
                            return

                        # Embed timing stamps into the command dict (UDP-style)
                        parsed["_t0"] = int(parsed.get("t0", 0))
                        parsed["_t1"] = int(parsed.get("t1", 0))
                        parsed["_t3"] = t3
                        parsed["_cmd_seq"] = int(parsed.get("cmd_seq", -1))
                        parsed["_size_bytes"] = size_bytes

                        with self._cmd_queue_lock:
                            if self._cmd_queue_bytes + size_bytes < self.max_queue_size_bytes:
                                try:
                                    self.cmd_in_queue.put_nowait(parsed)
                                    self._cmd_queue_bytes += size_bytes
                                except queue.Full:
                                    pass
                            else:
                                print(f"[Receiver] Queue size limit reached ({self._cmd_queue_bytes} bytes). Dropping packet.")
                    except Exception:
                        pass

        # ---------- Signaling via WebSocket ----------
        async with websockets.connect(self.signaling_url) as ws:
            # Register as "isaac"
            await ws.send(json.dumps({"register": "isaac"}))
            ack = json.loads(await ws.recv())
            print(f"[Signaling] Registered: {ack}")

            # Create offer
            offer = await self._pc.createOffer()
            await self._pc.setLocalDescription(offer)

            # Wait for ICE gathering to complete
            while self._pc.iceGatheringState != "complete":
                await asyncio.sleep(0.1)

            # Send offer (with gathered candidates baked in)
            await ws.send(json.dumps({
                "sdp": self._pc.localDescription.sdp,
                "type": self._pc.localDescription.type,
            }))
            print("[Signaling] 📤 Offer sent")

            # Wait for answer + ICE candidates
            async for raw in ws:
                msg = json.loads(raw)

                if "sdp" in msg and "type" in msg:
                    answer = RTCSessionDescription(sdp=msg["sdp"], type=msg["type"])
                    await self._pc.setRemoteDescription(answer)
                    print("[Signaling] 📥 Answer received")
                    
                    # Set connected flag AFTER we receive the answer and set remote description
                    self.connected.set()

                elif "candidate" in msg:
                    # Trickle ICE (optional — we bake candidates but Unity may trickle)
                    pass

                # Once connected, run the send loop
                if self.connected.is_set():
                    break

            # ----- State-sending loop -----
            print("[WebRTC] 🔄 Entering state-send loop")
            
            # Keep track of how many packets we've sent for debugging
            packets_sent = 0

            # Background task: listen for signaling messages (e.g. "bye") during state loop
            async def _watch_signaling():
                try:
                    async for raw in ws:
                        try:
                            msg = json.loads(raw)
                            if msg.get("bye"):
                                print("[Signaling] 📥 Received 'bye' from Unity — disconnecting")
                                self.connected.clear()
                                return
                        except Exception:
                            pass
                except websockets.ConnectionClosed:
                    print("[Signaling] WS closed during state loop")
                    self.connected.clear()

            sig_task = asyncio.ensure_future(_watch_signaling())
            
            while self._running and self.connected.is_set():
                try:
                    # Use get_nowait() so we don't block the asyncio event loop thread!
                    payload = self.state_out_queue.get_nowait()
                    if self._dc_state and self._dc_state.readyState == "open":
                        send_ms = self.clock.get_corrected_time_ms() if self.clock is not None else int(time.time() * 1000)
                        payload["t5_send"] = send_ms
                        payload_str = json.dumps(payload)
                        self._dc_state.send(payload_str)
                        packets_sent += 1
                        
                        # DEBUG: Print every 60th packet sent
                        if packets_sent % 60 == 0:
                            print(f"[WebRTC] 📤 Sent state packet #{packets_sent}: {payload_str[:50]}...")
                    else:
                        # DEBUG: If channel isn't open, print why
                        print(f"[WebRTC] ⚠️ Cannot send, channel state is: {self._dc_state.readyState if self._dc_state else 'None'}")
                            
                except queue.Empty:
                    pass
                # Allow the event loop to process incoming messages
                await asyncio.sleep(0.005) # Reduced sleep to 5ms for faster processing

        # Cancel signaling watcher
        sig_task.cancel()
        try:
            await sig_task
        except (asyncio.CancelledError, Exception):
            pass

        print("[WebRTC] 🔌 State-send loop ended — cleaning up session")

        # Cleanup
        await self._pc.close()
        self.connected.clear()

    # ------------------------------------------------------------------
    def enqueue_state(self, payload: dict):
        """Non-blocking enqueue; drops oldest if full."""
        try:
            self.state_out_queue.put_nowait(payload)
        except queue.Full:
            try:
                self.state_out_queue.get_nowait()
            except queue.Empty:
                pass
            self.state_out_queue.put_nowait(payload)

    def poll_commands(self):
        """Yield all pending command dicts."""
        while True:
            try:
                msg = self.cmd_in_queue.get_nowait()
                msg["_tq"] = self.clock.get_corrected_time_ms() if self.clock is not None else int(time.time() * 1000)
                yield msg
            except queue.Empty:
                break

    def consume_cmd_size_bytes(self, msg):
        size_bytes = msg.get("_size_bytes", 0)
        if size_bytes <= 0:
            return
        with self._cmd_queue_lock:
            self._cmd_queue_bytes = max(0, self._cmd_queue_bytes - size_bytes)

    def get_cmd_queue_bytes(self):
        with self._cmd_queue_lock:
            return self._cmd_queue_bytes

    def stop(self):
        self._running = False
        self.connected.clear()

# ==============================================================================
# 4. UR3 SERVER (Isaac Sim side — replaces the old UDP UR3UDPServer)
# ==============================================================================
class UR3WebRTCServer:
    def __init__(self):
        # --- Robot config (unchanged) ---
        self.ur3_path = "/ur3"
        self.physics_dt_denominator = 120  # physics dt = 1 / denominator seconds
        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        self.num_dof = 6
        self.target_q = np.zeros(self.num_dof, dtype=np.float64)
        self.frame_count = 0
        self.running = True

        # --- Chrony/System Clock ---
        self.ntp_sync = ChronyClock(check_sync=True)
        self.ntp_sync.start()

        # --- Packet/Queue Logging (UDP-style) ---
        self.log_writer = None
        self.log_file = None
        self.queue_log_writer = None
        self.queue_log_file = None
        self._init_logging()

        # --- Robot Setup ---
        try:
            self.ur3 = Articulation(self.ur3_path)
            self.ur3.initialize()
            print(f"✅ Robot found at: {self.ur3_path}")
        except:
            print(f"⚠️ Robot not found at {self.ur3_path}. Check your Prim Path!")
            self.ur3 = None

        self._configure_physics_timestep()

        # --- WebRTC Bridge ---
        self.bridge = WebRTCBridge(SIGNALING_URL, clock=self.ntp_sync)
        self.bridge.start()

        # --- UI ---
        self.slider_models = []
        self._build_ui()

        # --- omni update subscription ---
        self.subscription = omni.kit.app.get_app() \
            .get_update_event_stream() \
            .create_subscription_to_pop(self._on_update)

    def _configure_physics_timestep(self):
        try:
            hz = int(self.physics_dt_denominator)
            if hz <= 0:
                raise ValueError("physics_dt_denominator must be > 0")

            stage = omni.usd.get_context().get_stage()
            if stage is None:
                print("Physics dt not set: USD stage is not available yet.")
                return

            physics_scene_prim = None
            for prim in stage.Traverse():
                if prim.IsA(UsdPhysics.Scene):
                    physics_scene_prim = prim
                    break

            if physics_scene_prim is None:
                print("Physics dt not set: no UsdPhysics.Scene found in stage.")
                return

            physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene_prim)
            physx_scene_api.CreateTimeStepsPerSecondAttr().Set(hz)
            print(f"Physics timestep set to 1/{hz} s ({hz} Hz)")
        except Exception as e:
            print(f"Could not set physics timestep: {e}")

    def _init_logging(self):
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        base_dir = os.path.expanduser("~/Downloads/serverLogs")
        try:
            os.makedirs(base_dir, exist_ok=True)
        except Exception:
            pass

        packet_path = os.path.join(base_dir, f"isaac_packet_log_{timestamp}.csv")
        queue_path = os.path.join(base_dir, f"isaac_queue_log_{timestamp}.csv")

        try:
            self.log_file = open(packet_path, "w", newline="")
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow([
                "t3_recv",
                "tq_dequeue",
                "cmd_seq",
                "t0",
                "t1",
                "action_type",
                "action_data",
            ])
            print(f"Packet logging to: {packet_path}")
        except Exception as e:
            print(f"Could not open packet log file: {e}")
            self.log_file = None
            self.log_writer = None

        try:
            self.queue_log_file = open(queue_path, "w", newline="")
            self.queue_log_writer = csv.writer(self.queue_log_file)
            self.queue_log_writer.writerow([
                "utc_iso",
                "unix_ms",
                "packet_queue_size",
                "queue_bytes",
            ])
            print(f"Queue logging to: {queue_path}")
        except Exception as e:
            print(f"Could not open queue log file: {e}")
            self.queue_log_file = None
            self.queue_log_writer = None

    # --------------------------------------------------------------------------
    def _on_update(self, dt):
        if not self.running:
            return

        # Log queue depth each update (including zeros)
        if self.queue_log_writer:
            try:
                now_utc = datetime.now(timezone.utc)
                unix_ms = int(time.time() * 1000)
                self.queue_log_writer.writerow([
                    now_utc.isoformat().replace("+00:00", "Z"),
                    unix_ms,
                    self.bridge.cmd_in_queue.qsize(),
                    self.bridge.get_cmd_queue_bytes(),
                ])
                self.queue_log_file.flush()
            except Exception as e:
                print(f"Queue logging error: {e}")

        # --- 1. RECEIVE COMMANDS (via WebRTC DataChannel) ---
        # Each command in the queue carries its own _t0, _t1, _t3 so that
        # the state packet echoes the timestamps of the command it actually
        # processed — not a later command that arrived in the meantime.
        frame_has_cmd = False
        frame_t0 = 0
        frame_t1 = 0
        frame_t3 = 0
        frame_tq = 0
        frame_cmd_seq = self.bridge._frame_cmd_seq

        for msg in self.bridge.poll_commands():
            tq_dequeue = msg.get("_tq", self.ntp_sync.get_corrected_time_ms())
            if self.log_writer:
                try:
                    cmd_seq = msg.get("_cmd_seq")
                    t0 = msg.get("_t0")
                    t1 = msg.get("_t1")

                    if "delta" in msg:
                        action_type = "delta"
                        action_data = f"{msg['delta'][0]},{msg['delta'][1]}"
                    elif "joints" in msg:
                        action_type = "joints"
                        action_data = ",".join(str(x) for x in msg["joints"])
                    elif "handshake" in msg:
                        action_type = "handshake"
                        action_data = "1"
                    else:
                        action_type = "unknown"
                        action_data = json.dumps(msg)

                    self.log_writer.writerow([
                        msg.get("_t3", 0),
                        tq_dequeue,
                        cmd_seq,
                        t0,
                        t1,
                        action_type,
                        action_data,
                    ])
                    self.log_file.flush()
                except Exception:
                    pass

            self.bridge.consume_cmd_size_bytes(msg)
            if "delta" in msg:
                idx = int(msg["delta"][0])
                val = float(msg["delta"][1])
                self.target_q[idx] += val
                # Use this command's timestamps for the state response
                frame_t0 = msg.get("_t0", 0)
                frame_t1 = msg.get("_t1", 0)
                frame_t3 = msg.get("_t3", 0)
                frame_tq = tq_dequeue
                frame_cmd_seq = msg.get("_cmd_seq", frame_cmd_seq)
                frame_has_cmd = True
            elif "joints" in msg:
                self.target_q[:] = np.array(msg["joints"], dtype=np.float64)
                frame_t0 = msg.get("_t0", 0)
                frame_t1 = msg.get("_t1", 0)
                frame_t3 = msg.get("_t3", 0)
                frame_tq = tq_dequeue
                frame_cmd_seq = msg.get("_cmd_seq", frame_cmd_seq)
                frame_has_cmd = True
            elif "handshake" in msg:
                pass

        # Update per-frame timestamps only when a command was actually
        # processed this frame. If no command arrived, KEEP the previous
        # command's timestamps so the client can still compute latency.
        # The 'has_cmd' flag tells the client whether this packet is a
        # direct response to a new command or a repeat of the last one.
        if frame_has_cmd:
            self.bridge._frame_t0 = frame_t0
            self.bridge._frame_t1 = frame_t1
            self.bridge._frame_t3 = frame_t3
            self.bridge._frame_tq = frame_tq
            self.bridge._frame_cmd_seq = frame_cmd_seq
        # else: keep the previous frame's values unchanged

        # --- 2. PHYSICS UPDATE ---
        if self.ur3:
            self.ur3.set_joint_positions(self.target_q)

        # T4 = after physics step
        t4 = self.ntp_sync.get_corrected_time_ms()

        # Update UI sliders
        for i, model in enumerate(self.slider_models):
            if abs(model.as_float - self.target_q[i]) > 0.001:
                model.set_value(float(self.target_q[i]))

        # --- 3. SEND STATE (via WebRTC DataChannel) ---
        if self.bridge.connected.is_set():
            self.frame_count += 1
            
            # Get actual physics positions if robot is loaded, else fallback to target
            if self.ur3:
                current_q = self.ur3.get_joint_positions()
            else:
                current_q = self.target_q
                
            pos_list = current_q.tolist()
            pos_list = [0.0 if np.isnan(x) else x for x in pos_list]

            packet = {
                "seq": self.frame_count,
                "state": pos_list,
                "cmd_seq_echo": self.bridge._frame_cmd_seq,
                "t0_echo": self.bridge._frame_t0,
                "t1_echo": self.bridge._frame_t1,
                "t3_recv": self.bridge._frame_t3,    # Isaac command receive time (UTC ms)
                "tq_dequeue": self.bridge._frame_tq, # Isaac dequeue time (UTC ms)
                "t4_physics": t4,                   # Isaac physics done time (UTC ms)
            }
            
            # DEBUG: Print every 60th frame right before enqueueing
            if self.frame_count % 60 == 0:
                print(f"[Physics] 🔄 Enqueueing frame {self.frame_count}...")
                
            self.bridge.enqueue_state(packet)


    # --------------------------------------------------------------------------
    def _build_ui(self):
        self.window = ui.Window("WebRTC UR3 Control", width=350, height=450)
        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=8, style={"margin": 10}):
                    ui.Label("WebRTC SERVER RUNNING", style={"color": 0xFF00FF00, "font_size": 20})
                    ui.Label(f"Signaling: {SIGNALING_URL}", style={"color": 0xFFAAAAAA})

                    self.status_label = ui.Label(
                        "Waiting for Unity WebRTC peer...",
                        style={"color": 0xFF00FFFF},
                    )

                    ui.Spacer(height=10)

                    for i, name in enumerate(self.joint_names):
                        with ui.HStack(height=30):
                            ui.Label(name.replace("_joint", ""), width=120)
                            slider = ui.FloatSlider(min=-3.14, max=3.14)
                            slider.model.set_value(float(self.target_q[i]))
                            self.slider_models.append(slider.model)

                            def make_cb(idx):
                                return lambda m: self.target_q.__setitem__(idx, m.as_float)
                            slider.model.add_value_changed_fn(make_cb(i))

                    ui.Spacer(height=20)
                    ui.Button(
                        "STOP SERVER",
                        height=40,
                        clicked_fn=self.stop,
                        style={"background_color": 0xFF5555AA},
                    )

    def stop(self):
        self.running = False
        self.bridge.stop()
        self.subscription = None
        if self.ntp_sync:
            self.ntp_sync.stop()
        if self.window:
            self.window.visible = False
            self.window = None
        try:
            if self.log_file:
                self.log_file.close()
        except Exception:
            pass
        try:
            if self.queue_log_file:
                self.queue_log_file.close()
        except Exception:
            pass
        print("✅ WebRTC Server Stopped.")


# ==============================================================================
# START
# ==============================================================================
globals()["isaac_webrtc_server"] = UR3WebRTCServer()
