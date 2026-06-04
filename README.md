# Protocol Evaluation on Teleoperation

## Project Overview
This project evaluates the impact of underlying network protocol choices (TCP, UDP, and WebRTC) on the Quality of Experience (QoE) in robotic teleoperation. Teleoperation relies on two critically distinct data flows:
1. **Command Flow (Deltas):** From the client to the robot. This is non-idempotent data (e.g., "move joint by +0.05 rad"). If a packet is lost, the digital twin and real robot will experience permanent state desynchronization (drift).
2. **State Flow (Absolute Positions):** From the robot back to the client. This is idempotent and transient data. Stale data is useless; low latency visualization is prioritized over guaranteed delivery.

The project tests pure TCP/UDP flows alongside WebRTC DataChannels configured to mimic reliable (TCP-like) and unreliable (UDP-like) transfer characteristics. The experiments introduce real-world mobile network traces to evaluate how these protocols perform under degraded conditions (like limited throughput and packet loss).

## Project Structure
- **Server-side code:** All backend Python scripts and Isaac Sim server logic are present in `src/serverSrcCode`.
- **Client-side code:** The front-end Unity application is present in `src/clientSrcCode`.
- **Analysis scripts:** Python scripts and Jupyter notebooks for analyzing PCAP data, plotting drift, and generating summary tables are located in the `scripts/` folder.

## Setup Instructions

### Prerequisites
- **Unity:** Unity Editor (2022.3.2f1 recommended) for the client application.
- **Isaac Sim:** NVIDIA Omniverse Isaac Sim for the robot simulation side.
- **Python 3.8+:** For running the server-side logic and network emulation.
- **Python Packages:** `aiortc`, `websockets`, `numpy`, `pandas` (for analysis).

### Installation
1. Clone this repository:
   ```bash
   git clone https://github.com/Increshi/protocol_evaluation_on_teleoperation.git
   cd protocol_evaluation_on_teleoperation
   ```
2. Install Python dependencies:
   ```bash
   pip install aiortc websockets numpy pandas
   ```
3. Open the `src/clientSrcCode` project in Unity.

## How to Run

### 1. Pure UDP/TCP Flows
These flows use raw sockets without WebRTC overhead.
1. Run the respective server-side script in your Python/Isaac environment (e.g., waiting for pure TCP or pure UDP connections).
2. Launch the Unity client.
3. Select the test configuration and connect to the server IP.
4. Press the automated `continuous_send` button in the Unity UI to begin traffic generation.

### 2. WebRTC-Based Flows
1. Start the WebSocket signaling server first:
   ```bash
   python scripts/webrtc_signally_server.py
   ```
2. Start the Unity client and connect to the signaling server.
3. Start the Python WebRTC robot server (which connects to the signaling server to negotiate the peer-to-peer connection).
4. Wait for the WebRTC connection to establish, then press the `continuous_send` button in the Unity UI.

## Experiment Reproduction Steps

The experiments evaluate different protocol combinations under baseline (perfect network) and emulated 4G network conditions.

### Protocol Configurations
We test both Pure Socket and WebRTC-emulated combinations:
- **C-TCP / S-TCP:** Commands Reliable / State Reliable (High accuracy, High Network RTT)
- **C-TCP / S-UDP:** Commands Reliable / State Unreliable (High accuracy, Low Network RTT - **The Hybrid Ideal**)
- **C-UDP / S-TCP:** Commands Unreliable / State Reliable (Drift prone, High Network RTT)
- **C-UDP / S-UDP:** Commands Unreliable / State Unreliable (Drift prone, Low Network RTT)

*(Note: In WebRTC, C-TCP is emulated using `ordered=True, maxRetransmits=null`, and S-UDP is emulated using `ordered=False, maxRetransmits=0`).*

### Applying Network Degradation 
We utilize traffic control scripts (`tc_replay.py`) to emulate real-world 4G network traces provided in the `experiments/` directory.
- **Variant 1 (Baseline):** No packet loss, local network conditions.
- **Variant 2:** Applies `4G bicycle_001_throughput` data trace. Run `tc_replay.py` on client/server uplinks.
- **Variant 3:** Applies `4G bicycle_002_throughput` data trace (contains a 10s window of 0 throughput where 100% packet loss is induced).

To run a test block:
1. Start `tc_replay.py` applying the chosen network variant.
2. Ensure the Unity client code (from `src/clientSrcCode`) matches the target behavior (e.g., C-tcp/S-udp).
3. Connect and execute the `continuous_send` routine.
4. CSV logs recording timestamps (T6/T7) and calculated `Network RTT` will be generated locally. 


