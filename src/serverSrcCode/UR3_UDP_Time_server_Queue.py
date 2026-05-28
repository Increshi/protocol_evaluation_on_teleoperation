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
if "isaac_udp_server" in globals():
    print("Found existing server. Cleaning up...")
    globals()["isaac_udp_server"].stop()
    del globals()["isaac_udp_server"]

for obj in gc.get_objects():
    if isinstance(obj, socket.socket):
        try:
            obj.close()
        except:
            pass


# ==============================================================================
# 2. UDP SERVER CLASS
# ==============================================================================
class UR3UDPServer:
    def __init__(self):

        # --- Configuration ---
        self.listen_ip = "0.0.0.0"
        self.listen_port = 11020
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
        self.client_ip = None
        self.client_cmd_addr = None    # Where commands arrive from (send-side socket)
        self.client_state_addr = None  # Where state replies should go (receive-side socket)
        self.num_dof = 6
        self.target_q = np.zeros(self.num_dof, dtype=np.float64)
        self.frame_count = 0

        self.last_unity_t1 = None
        self.last_unity_t0 = None
        self.last_cmd_seq  = None  # cmd_seq echoed back so Unity can pair T1 ↔ response
        self.last_t3_recv = None   # Isaac UTC ms (chrony/system clock) — T3
        self.last_tq_dequeue = None  # Isaac UTC ms (chrony/system clock) — TQ
        self.last_t4_physics = 0   # Isaac UTC ms (chrony/system clock) — T4 (last frame)

        # --- Chrony/System Clock ---
        self.ntp_sync = ChronyClock(check_sync=True)
        self.ntp_sync.start()

        # --- Robot Setup ---
        try:
            self.ur3 = Articulation(self.ur3_path)
            self.ur3.initialize()
            print("Robot found at:", self.ur3_path)
        except:
            print("Robot not found at", self.ur3_path)
            self.ur3 = None

        self._configure_physics_timestep()

        # --- Networking ---
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.listen_ip, self.listen_port))
            self.sock.setblocking(False)
            print("UDP Active! Listening on port:", self.listen_port)
        except Exception as e:
            print("Socket Error:", e)
            self.running = False

        # Initialize packet log file
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        import os
        base_dir = os.path.expanduser("~/Downloads/serverLogs")  # or any folder you want
        self.log_path = os.path.join(base_dir, f"isaac_packet_log_{timestamp}.csv")
        #self.log_path = f"isaac_packet_log_{timestamp}.csv"
        try:
            self.log_file = open(self.log_path, 'w', newline='')
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow(['t3_recv', 'tq_dequeue', 'cmd_seq', 't0', 't1', 'action_type', 'action_data'])
            print(f"Packet logging to: {self.log_path}")
        except Exception as e:
            print(f"Could not open packet log file: {e}")
            self.log_file = None
            self.log_writer = None

        # Initialize queue-size log file (time series, includes zeros)
        self.queue_log_path = os.path.join(base_dir, f"isaac_queue_log_{timestamp}.csv")
        try:
            self.queue_log_file = open(self.queue_log_path, 'w', newline='')
            self.queue_log_writer = csv.writer(self.queue_log_file)
            self.queue_log_writer.writerow(['utc_iso', 'unix_ms', 'packet_queue_size', 'queue_bytes'])
            print(f"Queue logging to: {self.queue_log_path}")
        except Exception as e:
            print(f"Could not open queue log file: {e}")
            self.queue_log_file = None
            self.queue_log_writer = None

        # --- Packet Queue (Background Receiver) ---
        self.packet_queue = queue.Queue()
        self.max_queue_size_bytes = 10*1024 * 1024  # 1 MB
        self.current_queue_size_bytes = 0
        self._receiver_running = False
        self._receiver_thread = None

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
    # BACKGROUND RECEIVER THREAD
    # --------------------------------------------------------------------------
    def _estimate_packet_size(self, msg, addr):
        """Estimate the size in bytes of a packet item in the queue."""
        # JSON message + address tuple + t3_recv timestamp
        msg_size = sys.getsizeof(msg) + len(json.dumps(msg).encode('utf-8'))
        addr_size = sys.getsizeof(addr)
        t3_size = sys.getsizeof(0)  # Timestamp is an int
        overhead = sys.getsizeof({})  # Dict overhead for the queue item
        return msg_size + addr_size + t3_size + overhead

    def _receiver_loop(self):
        """Background thread: continuously receive UDP packets and add T3 timestamp."""
        while self._receiver_running:
            try:
                data, addr = self.sock.recvfrom(4096)

                # Parse JSON
                try:
                    msg = json.loads(data.decode("utf-8"))
                except Exception as e:
                    print(f"[Receiver] JSON parse error: {e}")
                    continue

                # Get T3 (UTC ms from system clock, typically chrony-disciplined)
                t3_recv = self.ntp_sync.get_corrected_time_ms()

                # Create packet item
                packet_item = {
                    'msg': msg,
                    'addr': addr,
                    't3_recv': t3_recv
                }

                # Estimate size
                packet_size = self._estimate_packet_size(msg, addr)

                # Check if adding this packet would exceed 1MB limit
                if self.current_queue_size_bytes + packet_size < self.max_queue_size_bytes:
                    self.packet_queue.put(packet_item)
                    self.current_queue_size_bytes += packet_size
                else:
                    print(f"[Receiver] Queue size limit reached ({self.current_queue_size_bytes} bytes). Dropping packet.")

            except BlockingIOError:
                # Non-blocking socket, no data available
                time.sleep(0.001)  # Small sleep to avoid busy-waiting
            except Exception as e:
                if self._receiver_running:
                    print(f"[Receiver] Error: {e}")
                break

    def _start_receiver_thread(self):
        """Start the background receiver thread."""
        if not self._receiver_running:
            self._receiver_running = True
            self._receiver_thread = threading.Thread(target=self._receiver_loop, daemon=True)
            self._receiver_thread.start()
            print("[Receiver] Background thread started")

    def _stop_receiver_thread(self):
        """Stop the background receiver thread."""
        self._receiver_running = False
        if self._receiver_thread and self._receiver_thread.is_alive():
            self._receiver_thread.join(timeout=2)
        print("[Receiver] Background thread stopped")

    # --------------------------------------------------------------------------
    # MAIN LOOP
    # --------------------------------------------------------------------------
    def _on_update(self, dt):
        if not self.running:
            return

        # Log queue depth each update (including when queue size is 0)
        if self.queue_log_writer:
            try:
                now_utc = datetime.now(timezone.utc)
                unix_ms = int(time.time() * 1000)
                self.queue_log_writer.writerow([
                    now_utc.isoformat().replace('+00:00', 'Z'),
                    unix_ms,
                    self.packet_queue.qsize(),
                    self.current_queue_size_bytes,
                ])
                self.queue_log_file.flush()
            except Exception as e:
                print(f"Queue logging error: {e}")

        # ---------------------------------------------------------------
        # 1. RECEIVE COMMANDS FROM QUEUE
        # ---------------------------------------------------------------
        while True:
            try:
                # Non-blocking get from queue
                packet_item = self.packet_queue.get_nowait()

                # TQ: time when the packet is dequeued for processing.
                tq_dequeue = self.ntp_sync.get_corrected_time_ms()

                msg = packet_item['msg']
                addr = packet_item['addr']
                t3_recv = packet_item['t3_recv']

                # Update queue size
                packet_size = self._estimate_packet_size(msg, addr)
                self.current_queue_size_bytes = max(0, self.current_queue_size_bytes - packet_size)

                # IP tracking
                if self.client_ip != addr[0]:
                    self.client_ip = addr[0]
                    print("Connected to Unity at:", addr[0], "port:", addr[1])

                # Log every received packet
                if self.log_writer:
                    cmd_seq = msg.get('cmd_seq')
                    t0 = msg.get('t0')
                    t1 = msg.get('t1')

                    if "delta" in msg:
                        action_type = 'delta'
                        action_data = f"{msg['delta'][0]},{msg['delta'][1]}"
                    elif "joints" in msg:
                        action_type = 'joints'
                        action_data = ','.join(str(x) for x in msg['joints'])
                    elif "handshake_rx" in msg:
                        action_type = 'handshake_rx'
                        action_data = str(msg['handshake_rx'])
                    else:
                        action_type = 'unknown'
                        action_data = json.dumps(msg)

                    self.log_writer.writerow([t3_recv, tq_dequeue, cmd_seq, t0, t1, action_type, action_data])
                    self.log_file.flush()

                # Route endpoint based on packet type:
                # - handshake_rx packets learn the receive-side port
                # - Command packets (delta, joints, etc.) learn the send-side port
                if isinstance(msg, dict) and "handshake_rx" in msg:
                    self.client_state_addr = addr
                else:
                    self.client_cmd_addr = addr

                # Always save T3 for every packet so Processing is always valid
                self.last_t3_recv = t3_recv
                self.last_tq_dequeue = tq_dequeue

                if "t0" in msg:
                    self.last_unity_t0 = msg["t0"]

                if "t1" in msg:
                    self.last_unity_t1 = msg["t1"]

                if "cmd_seq" in msg:
                    self.last_cmd_seq = msg["cmd_seq"]

                if "delta" in msg:
                    idx = int(msg["delta"][0])
                    val = float(msg["delta"][1])
                    self.target_q[idx] += val

                elif "joints" in msg:
                    self.target_q[:] = np.array(msg["joints"], dtype=np.float64)

            except queue.Empty:
                break
            except Exception as e:
                print("Queue processing error:", e)
                break

        # ---------------------------------------------------------------
        # 2. PHYSICS UPDATE
        # ---------------------------------------------------------------
        if self.ur3:
            self.ur3.set_joint_positions(self.target_q)

        t4_physics = self.ntp_sync.get_corrected_time_ms()   # T4 — UTC milliseconds
        self.last_t4_physics = t4_physics

        # Update UI sliders
        for i, model in enumerate(self.slider_models):
            if abs(model.as_float - self.target_q[i]) > 0.001:
                model.set_value(float(self.target_q[i]))

        # ---------------------------------------------------------------
        # 3. SEND STATE BACK
        # ---------------------------------------------------------------
        # Always send state to client_state_addr (receive-port endpoint)
        # Fall back to client_cmd_addr if state endpoint not yet learned
        target_addr = self.client_state_addr if self.client_state_addr else self.client_cmd_addr
        
        if target_addr:
            try:
                self.frame_count += 1
                pos_list = self.target_q.tolist()
                pos_list = [0.0 if np.isnan(x) else x for x in pos_list]

                t5_send = self.ntp_sync.get_corrected_time_ms()   # T5 — UTC milliseconds

                packet = {
                    "seq": self.frame_count,
                    "cmd_seq_echo": self.last_cmd_seq,  # echoed Unity cmd_seq for exact pairing
                    "t0_echo": self.last_unity_t0,      # echoed Unity T0 for exact pairing
                    "t1_echo": self.last_unity_t1,      # echoed Unity T1 for exact RTT pairing
                    "t3_recv": self.last_t3_recv,       # T3 — UTC ms
                    "tq_dequeue": self.last_tq_dequeue, # TQ — UTC ms
                    "t4_physics": t4_physics,           # T4 — UTC ms
                    "t5_send": t5_send,                 # T5 — UTC ms
                    "state": pos_list,
                }

                payload = json.dumps(packet).encode("utf-8")
                # Send to client_state_addr: the endpoint learned from receive-side handshake
                self.sock.sendto(payload, target_addr)

            except Exception:
                pass

    # --------------------------------------------------------------------------
    # UI BUILDER
    # --------------------------------------------------------------------------
    def _build_ui(self):
        self.window = ui.Window("UDP UR3 Control - Research Mode", width=350, height=450)

        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=8, style={"margin": 10}):

                    ui.Label(
                        "UDP SERVER RUNNING",
                        style={"color": 0xFF00FF00, "font_size": 20},
                    )

                    ui.Label(
                        "Listening: " + str(self.listen_port),
                        style={"color": 0xFFAAAAAA},
                    )

                    self.status_label = ui.Label(
                        "Waiting for Unity...",
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

    # --------------------------------------------------------------------------
    # STOP
    # --------------------------------------------------------------------------
    def stop(self):
        self.running = False
        self.subscription = None

        # Stop receiver thread
        self._stop_receiver_thread()

        # Stop NTP sync thread
        if self.ntp_sync:
            self.ntp_sync.stop()

        # Freeze robot at current position before shutdown
        if self.ur3:
            try:
                # Get current joint positions and set them to hold robot in place
                current_q = self.ur3.get_joint_positions()
                self.ur3.set_joint_positions(current_q)
                print("Robot frozen at position:", current_q)
            except Exception as e:
                print("Error freezing robot:", e)

        if self.window:
            self.window.visible = False
            self.window = None

        try:
            self.sock.close()
        except:
            pass

        if self.log_file:
            self.log_file.close()

        if self.queue_log_file:
            self.queue_log_file.close()

        print("Server Stopped.")


# ==============================================================================
# START SERVER
# ==============================================================================
globals()["isaac_udp_server"] = UR3UDPServer()















