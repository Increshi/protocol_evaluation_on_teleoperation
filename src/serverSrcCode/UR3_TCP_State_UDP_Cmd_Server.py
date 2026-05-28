import socket
import json
import time
import csv
import threading
import queue
import sys
import subprocess
from datetime import datetime, timezone
import numpy as np
import omni.kit.app
import omni.usd
import omni.ui as ui
import gc
from omni.isaac.core.articulations import Articulation
from pxr import PhysxSchema, UsdPhysics


# ==============================================================================
# CHRONY CLOCK WRAPPER
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
# 1. GLOBAL CLEANUP
# ==============================================================================
if "isaac_tcp_state_server" in globals():
    print("Found existing server. Cleaning up...")
    globals()["isaac_tcp_state_server"].stop()
    del globals()["isaac_tcp_state_server"]

for obj in gc.get_objects():
    if isinstance(obj, socket.socket):
        try:
            obj.close()
        except Exception:
            pass


# ==============================================================================
# 2. UDP CMD + TCP STATE SERVER
# ==============================================================================
class UR3UdpCmdTcpStateServer:
    def __init__(self):
        # --- Configuration ---
        self.listen_ip = "0.0.0.0"
        self.cmd_port = 11020   # UDP commands
        self.state_port = 11021  # TCP state
        self.physics_dt_denominator = 120  # physics dt = 1 / denominator seconds

        self.ur3_path = "/ur3"
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # --- State ---
        self.running = True
        self.cmd_client_addr = None
        self.state_conn = None
        self.num_dof = 6
        self.target_q = np.zeros(self.num_dof, dtype=np.float64)
        self.frame_count = 0

        self.last_unity_t1 = None
        self.last_unity_t0 = None
        self.last_cmd_seq = None
        self.last_t3_recv = None
        self.last_tq_dequeue = None
        self.last_t4_physics = 0

        self._target_q_lock = threading.Lock()
        self._pending_ts_lock = threading.Lock()
        self._pending_ts = None

        # --- Chrony/System Clock ---
        self.ntp_sync = ChronyClock(check_sync=True)
        self.ntp_sync.start()

        # --- Robot Setup ---
        try:
            self.ur3 = Articulation(self.ur3_path)
            self.ur3.initialize()
            print("Robot found at:", self.ur3_path)
        except Exception:
            print("Robot not found at", self.ur3_path)
            self.ur3 = None

        self._configure_physics_timestep()

        # --- UDP Command Socket ---
        try:
            self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.cmd_sock.bind((self.listen_ip, self.cmd_port))
            self.cmd_sock.setblocking(False)
            print("UDP Cmd Active! Listening on port:", self.cmd_port)
        except Exception as e:
            print("Cmd socket error:", e)
            self.running = False

        # --- TCP State Socket ---
        self.state_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.state_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.state_sock.bind((self.listen_ip, self.state_port))
            self.state_sock.listen(1)
            print("TCP State Active! Listening on port:", self.state_port)
        except Exception as e:
            print("State socket error:", e)
            self.running = False

        # --- Packet Queue (Background Receiver) ---
        self.packet_queue = queue.Queue()
        self.max_queue_size_bytes = 10 * 1024 * 1024
        self.current_queue_size_bytes = 0
        self._receiver_running = False
        self._receiver_thread = None

        # --- TCP State Sender Queue ---
        self._state_send_q: queue.Queue[bytes] = queue.Queue(maxsize=4)
        self._state_sender_thread = None
        self._state_alive_evt = threading.Event()

        # --- Logging ---
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        import os
        base_dir = os.path.expanduser("~/Downloads/serverLogs")
        os.makedirs(base_dir, exist_ok=True)

        self.log_path = os.path.join(base_dir, f"isaac_tcp_state_packet_log_{timestamp}.csv")
        try:
            self.log_file = open(self.log_path, "w", newline="")
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow(["t3_recv", "tq_dequeue", "cmd_seq", "t0", "t1", "action_type", "action_data"])
            print(f"Packet logging to: {self.log_path}")
        except Exception as e:
            print(f"Could not open packet log file: {e}")
            self.log_file = None
            self.log_writer = None

        self.queue_log_path = os.path.join(base_dir, f"isaac_tcp_state_queue_log_{timestamp}.csv")
        try:
            self.queue_log_file = open(self.queue_log_path, "w", newline="")
            self.queue_log_writer = csv.writer(self.queue_log_file)
            self.queue_log_writer.writerow(["utc_iso", "unix_ms", "packet_queue_size", "queue_bytes"])
            print(f"Queue logging to: {self.queue_log_path}")
        except Exception as e:
            print(f"Could not open queue log file: {e}")
            self.queue_log_file = None
            self.queue_log_writer = None

        # --- UI ---
        self.slider_models = []
        self._build_ui()

        self.subscription = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(self._on_update)
        )

        # Start background receiver thread
        self._start_receiver_thread()

        # Start TCP accept thread for state
        threading.Thread(target=self._state_accept_loop, daemon=True).start()

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

    # --------------------------------------------------------------------------
    # BACKGROUND RECEIVER THREAD (UDP CMD)
    # --------------------------------------------------------------------------
    def _estimate_packet_size(self, msg, addr):
        msg_size = sys.getsizeof(msg) + len(json.dumps(msg).encode("utf-8"))
        addr_size = sys.getsizeof(addr)
        t3_size = sys.getsizeof(0)
        overhead = sys.getsizeof({})
        return msg_size + addr_size + t3_size + overhead

    def _receiver_loop(self):
        while self._receiver_running:
            try:
                data, addr = self.cmd_sock.recvfrom(4096)

                try:
                    msg = json.loads(data.decode("utf-8"))
                except Exception as e:
                    print(f"[Receiver] JSON parse error: {e}")
                    continue

                t3_recv = self.ntp_sync.get_corrected_time_ms()
                packet_item = {
                    "msg": msg,
                    "addr": addr,
                    "t3_recv": t3_recv,
                }

                packet_size = self._estimate_packet_size(msg, addr)

                if self.current_queue_size_bytes + packet_size < self.max_queue_size_bytes:
                    self.packet_queue.put(packet_item)
                    self.current_queue_size_bytes += packet_size
                else:
                    print(f"[Receiver] Queue size limit reached ({self.current_queue_size_bytes} bytes). Dropping packet.")

            except BlockingIOError:
                time.sleep(0.001)
            except Exception as e:
                if self._receiver_running:
                    print(f"[Receiver] Error: {e}")
                break

    def _start_receiver_thread(self):
        if not self._receiver_running:
            self._receiver_running = True
            self._receiver_thread = threading.Thread(target=self._receiver_loop, daemon=True)
            self._receiver_thread.start()
            print("[Receiver] Background thread started")

    def _stop_receiver_thread(self):
        self._receiver_running = False
        if self._receiver_thread and self._receiver_thread.is_alive():
            self._receiver_thread.join(timeout=2)
        print("[Receiver] Background thread stopped")

    # --------------------------------------------------------------------------
    # TCP STATE ACCEPT + SENDER
    # --------------------------------------------------------------------------
    def _state_accept_loop(self):
        while self.running:
            try:
                client, addr = self.state_sock.accept()
            except Exception:
                break

            print(f"TCP State client connected: {addr}")
            self.state_conn = client
            self._state_alive_evt = threading.Event()
            self._state_alive_evt.set()

            try:
                client.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            except Exception:
                pass

            self._state_sender_thread = threading.Thread(
                target=self._state_sender_loop, args=(client, self._state_alive_evt), daemon=True
            )
            self._state_sender_thread.start()

            try:
                while self.running and self._state_alive_evt.is_set():
                    time.sleep(0.2)
            finally:
                self._state_alive_evt.clear()
                self.state_conn = None
                try:
                    client.close()
                except Exception:
                    pass
                print("TCP State client disconnected.")

    def _state_sender_loop(self, conn, alive_evt):
        try:
            while self.running and self.state_conn is conn and alive_evt.is_set():
                try:
                    payload = self._state_send_q.get(timeout=0.5)
                except queue.Empty:
                    continue
                try:
                    conn.sendall(payload)
                except (BrokenPipeError, ConnectionResetError, OSError) as e:
                    print(f"[state_sender] connection lost: {type(e).__name__}: {e}")
                    alive_evt.clear()
                    break
                except Exception as e:
                    print(f"[state_sender] sendall error: {type(e).__name__}: {e}")
        finally:
            pass

    # --------------------------------------------------------------------------
    # MAIN LOOP
    # --------------------------------------------------------------------------
    def _on_update(self, dt):
        if not self.running:
            return

        if self.queue_log_writer:
            try:
                now_utc = datetime.now(timezone.utc)
                unix_ms = int(time.time() * 1000)
                self.queue_log_writer.writerow([
                    now_utc.isoformat().replace("+00:00", "Z"),
                    unix_ms,
                    self.packet_queue.qsize(),
                    self.current_queue_size_bytes,
                ])
                self.queue_log_file.flush()
            except Exception as e:
                print(f"Queue logging error: {e}")

        while True:
            try:
                packet_item = self.packet_queue.get_nowait()
                tq_dequeue = self.ntp_sync.get_corrected_time_ms()

                msg = packet_item["msg"]
                addr = packet_item["addr"]
                t3_recv = packet_item["t3_recv"]

                packet_size = self._estimate_packet_size(msg, addr)
                self.current_queue_size_bytes = max(0, self.current_queue_size_bytes - packet_size)

                if self.cmd_client_addr != addr:
                    self.cmd_client_addr = addr
                    print("UDP Cmd connected to Unity at:", addr[0], "port:", addr[1])

                if self.log_writer:
                    cmd_seq = msg.get("cmd_seq")
                    t0 = msg.get("t0")
                    t1 = msg.get("t1")

                    if "delta" in msg:
                        action_type = "delta"
                        action_data = f"{msg['delta'][0]},{msg['delta'][1]}"
                    elif "joints" in msg:
                        action_type = "joints"
                        action_data = ",".join(str(x) for x in msg["joints"])
                    else:
                        action_type = "unknown"
                        action_data = json.dumps(msg)

                    self.log_writer.writerow([t3_recv, tq_dequeue, cmd_seq, t0, t1, action_type, action_data])
                    self.log_file.flush()

                if "t0" in msg:
                    self.last_unity_t0 = msg["t0"]
                if "t1" in msg:
                    self.last_unity_t1 = msg["t1"]
                if "cmd_seq" in msg:
                    self.last_cmd_seq = msg["cmd_seq"]

                if "delta" in msg:
                    idx = int(msg["delta"][0])
                    val = float(msg["delta"][1])
                    with self._target_q_lock:
                        self.target_q[idx] += val
                elif "joints" in msg:
                    with self._target_q_lock:
                        self.target_q[:] = np.array(msg["joints"], dtype=np.float64)

                with self._pending_ts_lock:
                    self._pending_ts = {
                        "t0": int(msg.get("t0", 0)),
                        "t1": int(msg.get("t1", 0)),
                        "cmd_seq": int(msg.get("cmd_seq", 0)),
                        "t3": t3_recv,
                        "tq": tq_dequeue,
                    }

            except queue.Empty:
                break
            except Exception as e:
                print("Queue processing error:", e)
                break

        # Physics update
        t4_physics = self.ntp_sync.get_corrected_time_ms()
        self.last_t4_physics = t4_physics

        if self.ur3:
            with self._target_q_lock:
                q_snapshot = self.target_q.copy()
            self.ur3.set_joint_positions(q_snapshot)
        else:
            with self._target_q_lock:
                q_snapshot = self.target_q.copy()

        for i, model in enumerate(self.slider_models):
            if abs(model.as_float - q_snapshot[i]) > 0.001:
                model.set_value(float(q_snapshot[i]))

        # Send TCP state (newline-delimited JSON)
        if self.state_conn:
            try:
                self.frame_count += 1
                safe_pos = [0.0 if np.isnan(x) else float(x) for x in q_snapshot]
                t5_send = self.ntp_sync.get_corrected_time_ms()

                with self._pending_ts_lock:
                    ts = self._pending_ts or {}
                    self._pending_ts = None

                payload = (json.dumps({
                    "seq": self.frame_count,
                    "cmd_seq_echo": ts.get("cmd_seq", 0),
                    "t0_echo": ts.get("t0", 0),
                    "t1_echo": ts.get("t1", 0),
                    "t3_recv": ts.get("t3", 0),
                    "tq_dequeue": ts.get("tq", 0),
                    "t4_physics": t4_physics,
                    "t5_send": t5_send,
                    "state": safe_pos,
                }) + "\n").encode("utf-8")

                try:
                    self._state_send_q.put_nowait(payload)
                except queue.Full:
                    try:
                        self._state_send_q.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        self._state_send_q.put_nowait(payload)
                    except queue.Full:
                        pass
            except Exception as e:
                print(f"State prep error: {type(e).__name__}: {e}")

    # --------------------------------------------------------------------------
    # UI BUILDER
    # --------------------------------------------------------------------------
    def _build_ui(self):
        self.window = ui.Window("UDP Cmd + TCP State UR3 Control", width=350, height=450)

        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=8, style={"margin": 10}):
                    ui.Label("SERVER RUNNING", style={"color": 0xFF00FF00, "font_size": 20})
                    ui.Label("Cmd UDP: " + str(self.cmd_port), style={"color": 0xFFAAAAAA})
                    ui.Label("State TCP: " + str(self.state_port), style={"color": 0xFFAAAAAA})

                    ui.Spacer(height=10)

                    for i, name in enumerate(self.joint_names):
                        with ui.HStack(height=30):
                            ui.Label(name.replace("_joint", ""), width=120)
                            slider = ui.FloatSlider(min=-3.14, max=3.14)
                            with self._target_q_lock:
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

    # --------------------------------------------------------------------------
    # STOP
    # --------------------------------------------------------------------------
    def stop(self):
        self.running = False
        self.subscription = None

        self._stop_receiver_thread()

        if self.ntp_sync:
            self.ntp_sync.stop()

        if self.ur3:
            try:
                current_q = self.ur3.get_joint_positions()
                self.ur3.set_joint_positions(current_q)
                print("Robot frozen at position:", current_q)
            except Exception as e:
                print("Error freezing robot:", e)

        if self.window:
            self.window.visible = False
            self.window = None

        for s in (self.cmd_sock, self.state_sock):
            try:
                s.close()
            except Exception:
                pass

        if self.state_conn:
            try:
                self.state_conn.close()
            except Exception:
                pass

        if self.log_file:
            self.log_file.close()
        if self.queue_log_file:
            self.queue_log_file.close()

        print("Server Stopped.")


# ==============================================================================
# START SERVER
# ==============================================================================
globals()["isaac_tcp_state_server"] = UR3UdpCmdTcpStateServer()
