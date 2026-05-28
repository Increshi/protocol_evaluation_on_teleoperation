import socket
import json
import threading
import queue
import csv
import sys
from datetime import datetime, timezone
import numpy as np
import omni.kit.app
import omni.ui as ui
import time
import subprocess
import gc
import traceback
import os
from omni.isaac.core.articulations import Articulation

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
# 1. GLOBAL CLEANUP (Prevents "Port In Use" errors)
# ==============================================================================
if "ctcp_sudp_server" in globals():
    print("♻️ Found existing server. Cleaning up...")
    try:
        globals()["ctcp_sudp_server"].stop()
    except Exception:
        pass
    del globals()["ctcp_sudp_server"]
    time.sleep(0.5)

for obj in gc.get_objects():
    if isinstance(obj, socket.socket):
        try:
            obj.close()
        except Exception:
            pass

# ==============================================================================
# 2. TCP CMD + UDP STATE SERVER
# ==============================================================================
class UR3CTCPSUDPServer:
    def __init__(self):
        # --- Configuration ---
        self.listen_ip = "0.0.0.0"
        self.cmd_port = 11020  # TCP: Receive Commands
        self.state_port = 11021  # UDP: Send State

        self.ur3_path = "/ur3"
        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]

        # --- State ---
        self.running = True
        self.cmd_conn = None
        self.num_dof = 6
        self.target_q = np.zeros(self.num_dof, dtype=np.float64)
        self.frame_count = 0
        self._pending_ts = None
        self._cmd_sock = None
        self._cmd_packet_q = queue.Queue()
        self._max_cmd_queue_bytes = 10 * 1024 * 1024
        self._cmd_queue_bytes = 0
        self._receiver_running = False
        self._receiver_thread = None
        self._cmd_conn_addr = None
        self._state_client_addr = None
        self._target_q_lock = threading.Lock()
        self._pending_ts_lock = threading.Lock()

        # UDP socket for outbound state
        self._state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # --- Chrony/System Clock ---
        self.ntp_sync = ChronyClock(check_sync=True)
        self.ntp_sync.start()

        # --- Robot Setup ---
        try:
            self.ur3 = Articulation(self.ur3_path)
            self.ur3.initialize()
            print(f"✅ Robot found at: {self.ur3_path}")
        except Exception:
            print(f"⚠️ Robot not found at {self.ur3_path}. Check Prim Path!")
            self.ur3 = None

        # --- UI Setup ---
        self.slider_models = []
        self._build_ui()

        # --- Logging ---
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        base_dir = os.path.expanduser("~/Downloads/serverLogs")
        os.makedirs(base_dir, exist_ok=True)
        self.log_path = os.path.join(base_dir, f"isaac_ctcp_sudp_packet_log_{timestamp}.csv")
        try:
            self.log_file = open(self.log_path, "w", newline="")
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow(["t3_recv", "tq_dequeue", "cmd_seq", "t0", "t1", "action_type", "action_data"])
            print(f"Packet logging to: {self.log_path}")
        except Exception as e:
            print(f"Could not open packet log file: {e}")
            self.log_file = None
            self.log_writer = None

        self.queue_log_path = os.path.join(base_dir, f"isaac_ctcp_sudp_queue_log_{timestamp}.csv")
        try:
            self.queue_log_file = open(self.queue_log_path, "w", newline="")
            self.queue_log_writer = csv.writer(self.queue_log_file)
            self.queue_log_writer.writerow(["utc_iso", "unix_ms", "packet_queue_size", "queue_bytes"])
            print(f"Queue logging to: {self.queue_log_path}")
        except Exception as e:
            print(f"Could not open queue log file: {e}")
            self.queue_log_file = None
            self.queue_log_writer = None

        # --- Start Networking Threads ---
        threading.Thread(target=self.listen_cmd, daemon=True).start()
        print(f"🚀 CTCP+SUDP Server Active! (Cmd TCP:{self.cmd_port}, State UDP:{self.state_port})")

        # --- Start Physics Loop ---
        self.sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_update)

    # --------------------------------------------------------------------------
    # THREAD: RECEIVE COMMANDS (TCP Port 11020)
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
                    break
                print(f"📥 Cmd client connected: {addr}")
                self.cmd_conn = client
                self._cmd_conn_addr = addr
                self._state_client_addr = (addr[0], self.state_port)
                self._start_receiver_thread(client)

                try:
                    while self.running and self.cmd_conn is client:
                        time.sleep(0.2)
                finally:
                    self._stop_receiver_thread()
                    self.cmd_conn = None
                    self._cmd_conn_addr = None
                    self._state_client_addr = None
                    try:
                        client.close()
                    except Exception:
                        pass
                    print("📥 Cmd client disconnected.")
        except Exception as e:
            if self.running:
                print(f"📥 Cmd socket error: {type(e).__name__}: {e}")
        finally:
            sock.close()

    def _estimate_packet_size(self, msg, addr):
        msg_size = sys.getsizeof(msg) + len(json.dumps(msg).encode("utf-8"))
        addr_size = sys.getsizeof(addr)
        t3_size = sys.getsizeof(0)
        overhead = sys.getsizeof({})
        return msg_size + addr_size + t3_size + overhead

    def _start_receiver_thread(self, conn):
        if not self._receiver_running:
            self._receiver_running = True
            self._receiver_thread = threading.Thread(
                target=self._receiver_loop, args=(conn,), daemon=True
            )
            self._receiver_thread.start()

    def _stop_receiver_thread(self):
        self._receiver_running = False
        if self._receiver_thread and self._receiver_thread.is_alive():
            self._receiver_thread.join(timeout=1.0)
        self._receiver_thread = None

    def _receiver_loop(self, conn):
        buffer = ""
        try:
            conn.settimeout(0.5)
        except Exception:
            pass
        while self._receiver_running and self.running and self.cmd_conn is conn:
            try:
                chunk = conn.recv(1024)
            except socket.timeout:
                continue
            except (ConnectionResetError, BrokenPipeError, OSError):
                print("📥 Cmd client connection lost.")
                break

            if not chunk:
                print("📥 Cmd client closed connection (EOF).")
                break

            buffer += chunk.decode("utf-8", errors="replace")
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line)
                    t3_recv = self.ntp_sync.get_corrected_time_ms()
                    packet_item = {
                        "msg": msg,
                        "addr": self._cmd_conn_addr,
                        "t3_recv": t3_recv,
                    }
                    packet_size = self._estimate_packet_size(msg, self._cmd_conn_addr)
                    if self._cmd_queue_bytes + packet_size < self._max_cmd_queue_bytes:
                        self._cmd_packet_q.put(packet_item)
                        self._cmd_queue_bytes += packet_size
                    else:
                        print(f"📥 [cmd_receiver] queue limit reached ({self._cmd_queue_bytes} bytes). Dropping packet.")
                except Exception as pe:
                    print(f"📥 [cmd_receiver] parse error: {pe}")

        self._receiver_running = False

    # --------------------------------------------------------------------------
    # MAIN LOOP: PHYSICS & UI UPDATE
    # --------------------------------------------------------------------------
    _on_update_send_err_time = 0.0

    def _on_update(self, dt):
        if not self.running:
            return

        # Log queue depth each update (including when queue size is 0)
        if self.queue_log_writer:
            try:
                now_utc = datetime.now(timezone.utc)
                unix_ms = int(time.time() * 1000)
                self.queue_log_writer.writerow([
                    now_utc.isoformat().replace("+00:00", "Z"),
                    unix_ms,
                    self._cmd_packet_q.qsize(),
                    self._cmd_queue_bytes,
                ])
                self.queue_log_file.flush()
            except Exception as e:
                print(f"Queue logging error: {e}")

        # 0. Drain command queue and apply latest commands
        while True:
            try:
                packet_item = self._cmd_packet_q.get_nowait()

                tq_dequeue = self.ntp_sync.get_corrected_time_ms()
                msg = packet_item["msg"]
                addr = packet_item["addr"]
                t3_recv = packet_item["t3_recv"]

                packet_size = self._estimate_packet_size(msg, addr)
                self._cmd_queue_bytes = max(0, self._cmd_queue_bytes - packet_size)

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

                if "joints" in msg:
                    with self._target_q_lock:
                        self.target_q[:] = np.array(msg["joints"], dtype=np.float64)
                elif "delta" in msg:
                    idx = int(msg["delta"][0])
                    val = float(msg["delta"][1])
                    with self._target_q_lock:
                        self.target_q[idx] += val
                else:
                    continue

                t0 = int(msg.get("t0", 0))
                t1 = int(msg.get("t1", 0))
                cmd_seq = int(msg.get("cmd_seq", 0))
                with self._pending_ts_lock:
                    self._pending_ts = {
                        "t0": t0,
                        "t1": t1,
                        "cmd_seq": cmd_seq,
                        "t3": t3_recv,
                        "tq": tq_dequeue,
                    }

            except queue.Empty:
                break
            except Exception as e:
                print(f"Cmd queue processing error: {e}")
                break

        # T4 = physics applied (UTC ms, chrony-disciplined)
        t4 = self.ntp_sync.get_corrected_time_ms()

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

        # 3. Send State over UDP (If TCP cmd client connected)
        if self._state_client_addr:
            try:
                if self.ur3:
                    try:
                        actual_pos = self.ur3.get_joint_positions()
                        safe_pos = [0.0 if np.isnan(x) else float(x) for x in actual_pos]
                    except Exception:
                        with self._target_q_lock:
                            safe_pos = [0.0 if np.isnan(x) else float(x) for x in self.target_q.copy()]
                else:
                    with self._target_q_lock:
                        safe_pos = [0.0 if np.isnan(x) else float(x) for x in self.target_q.copy()]

                t5 = self.ntp_sync.get_corrected_time_ms()
                with self._pending_ts_lock:
                    ts = self._pending_ts or {}
                    self._pending_ts = None

                self.frame_count += 1

                payload = json.dumps({
                    "seq": self.frame_count,
                    "cmd_seq_echo": ts.get("cmd_seq", 0),
                    "t0_echo": ts.get("t0", 0),
                    "t1_echo": ts.get("t1", 0),
                    "t3_recv": ts.get("t3", 0),
                    "tq_dequeue": ts.get("tq", 0),
                    "t4_physics": t4,
                    "t5_send": t5,
                    "state": safe_pos,
                }).encode("utf-8")

                self._state_sock.sendto(payload, self._state_client_addr)
            except Exception as e:
                now = time.time()
                if now - UR3CTCPSUDPServer._on_update_send_err_time >= 1.0:
                    UR3CTCPSUDPServer._on_update_send_err_time = now
                    print(f"📡 [_on_update] UDP state send FAILED — {type(e).__name__}: {e}")
                    traceback.print_exc()

    # --------------------------------------------------------------------------
    # UI BUILDER
    # --------------------------------------------------------------------------
    def _build_ui(self):
        self.window = ui.Window("CTCP + SUDP UR3 Control", width=350, height=450)
        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=8, style={"margin": 10}):
                    ui.Label("SERVER RUNNING", style={"color": 0xFF00FF00, "font_size": 20})

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

        if self.sub is not None:
            try:
                self.sub.unsubscribe()
            except Exception:
                pass
            self.sub = None

        if self._cmd_sock is not None:
            try:
                self._cmd_sock.close()
            except Exception:
                pass
        self._cmd_sock = None

        if self.cmd_conn is not None:
            try:
                self.cmd_conn.close()
            except Exception:
                pass
        self.cmd_conn = None

        try:
            self._state_sock.close()
        except Exception:
            pass

        self._stop_receiver_thread()

        while not self._cmd_packet_q.empty():
            try:
                self._cmd_packet_q.get_nowait()
            except queue.Empty:
                break

        if self.log_file:
            self.log_file.close()
        if self.queue_log_file:
            self.queue_log_file.close()

        if self.window:
            self.window.visible = False
            self.window = None

        if hasattr(self, "ntp_sync") and self.ntp_sync:
            self.ntp_sync.stop()

        print("✅ Server Stopped.")

# ==============================================================================
# START
# ==============================================================================
globals()["ctcp_sudp_server"] = UR3CTCPSUDPServer()
