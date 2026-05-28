using UnityEngine;
using UnityEngine.UI;
using System;
using System.Text;
using System.Threading;
using System.Globalization;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.IO;
using System.Net;
using System.Net.Sockets;
using TMPro;

// ─────────────────────────────────────────────────────────────────
// Unity WebRTC package  (com.unity.webrtc)
// Install via Package Manager → Add package by name:
//     com.unity.webrtc
// ─────────────────────────────────────────────────────────────────
using Unity.WebRTC;

// ─────────────────────────────────────────────────────────────────
// NativeWebSocket for signaling  (WebSocket client for Unity)
// Install via Package Manager → Add package from git URL:
//     https://github.com/endel/NativeWebSocket.git#upm
// ─────────────────────────────────────────────────────────────────
using NativeWebSocket;

/// <summary>
/// Drop-in replacement for IsaacUR3Client that uses WebRTC DataChannels
/// instead of raw UDP for robot-state streaming.
///
/// PUBLIC API is the same:
///   • IsaacUR3WebRTCClient.JointAnglesRad[0..5]
///   • IsaacUR3WebRTCClient.LastPacketTimestamp
/// So your existing "link" scripts need only reference this class instead.
/// </summary>
public class IsaacUR3WebRTCClient : MonoBehaviour
{
    // ──────── SHARED DATA (same public contract as before) ────────
    public static float[] JointAnglesRad = new float[6];
    public static long LastPacketTimestamp = 0;

    // Timing statics mirrored from UDP client so shared loggers can read them
    public static long T0_InputCapture     = 0;
    public static long T1_PacketSent       = 0;
    public static long T6_UnityReceived    = 0;
    public static long T3_IsaacReceived    = 0;
    public static long TQ_IsaacDequeue     = 0;
    public static long T4_PhysicsApplied   = 0;
    public static long T5_IsaacSend        = 0;
    public static long T1_ForLastResponse  = 0;
    public static int  LastReceivedSeqPublic = -1;
    public static int  LastReceivedCmdSeq    = -1;
    public static float CurrentJitterMs      = 0f;

    // --- Drift Analytics Tracking ---
    public static float[] ExpectedJointAngles = new float[6];
    public static bool ExpectedInitialized = false;
    public static ConcurrentDictionary<int, float[]> CmdToExpected = new ConcurrentDictionary<int, float[]>();

    public static readonly object StateLock = new object();
    public static IsaacState LatestState = null;

    // ──────── UI ────────
    [Header("UI Assignments")]
    public TMP_InputField ipInput;
    public TMP_Text statusText;
    public GameObject connectionPanel;

    [Header("Disconnect UI (optional)")]
    [Tooltip("Assign a Disconnect button — its onClick will be wired automatically")]
    public UnityEngine.UI.Button disconnectButton;

    // ──────── Settings ────────
    [Header("WebRTC Settings")]
    [Tooltip("WebSocket URL of the signaling server")]
    public string signalingUrl = "ws://10.9.71.137:8765";

    [Header("Calibration")]
    public float[] visualOffsets = new float[6];
    public float[] jointSigns = new float[] { 1, 1, 1, 1, 1, 1 };

    // ──────── Internal ────────
    [Serializable]
    public class IsaacState
    {
        public int seq;
        public int cmd_seq_echo;
        public long t0_echo;     // Unity input capture time  (ms, Unity clock)
        public long t1_echo;     // Unity command send time   (ms, Unity clock)
        public long t3_recv;     // Isaac command receive time (UTC ms)
        public long tq_dequeue;  // Isaac queue dequeue time  (UTC ms)
        public long t4_physics;  // Isaac physics done time   (UTC ms)
        public long t5_send;     // Isaac state send time     (UTC ms)
        public long t6_unity;    // Unity receive time (ms) — filled client-side
        public float[] state;
        // Optional fields retained for backward compatibility with older WebRTC payloads
        public long ts;          // sender wall-clock ms (optional)
        public bool has_cmd;     // optional
    }

    // ---- Per-packet timing: receive → Update → LateUpdate ----
    private struct PendingPacket
    {
        public IsaacState data;
        public long t6;   // Unity receive time (ms, Unity clock)
    }

    public struct PacketHistoryEntry
    {
        public DateTime TimestampUtc;
        public int Seq;
        public int CmdSeqEcho;
        public bool HasCommand;
        public long ServerTimestampMs;
        public long T0Ns;
        public long T1Ns;
        public long T3Ns;
        public long T4Ns;
        public long T5Ns;
        public long T6Ns;
        public long T7Ns;
        public double RttMs;
        public double ProcessingMs;
        public double NetworkEstMs;
        public double RenderDelayMs;
        public double MotionToPhotonMs;
        public float JitterMs;
    }

    private WebSocket _ws;
    private RTCPeerConnection _pc;
    private RTCDataChannel _dcCommands;   // we create – for sending joystick cmds
    private RTCDataChannel _dcState;      // we receive – MUST keep reference to prevent garbage collection!
    private int _lastReceivedSeq = -1;
    private bool _isConnected = false;

    // Thread-safe queue for received packets (stamped with t6)
    private ConcurrentQueue<PendingPacket> _packetQueue = new ConcurrentQueue<PendingPacket>();
    private PendingPacket _pendingForLate;
    private bool _hasPendingLate = false;

    private int _cmdSeq = 0;
    private int _lastConsumedCmdSeq = -1;

    private Coroutine _continuousSendRoutine = null;
    private bool _continuousSendActive = false;
    private int _continuousIndex = 0;
    private float _continuousAmount = 0f;
   

    // ──────── Packet history storage ────────
    private const int MaxPacketHistoryEntries = 2048;
    private static readonly object _packetHistoryLock = new object();
    private static readonly Queue<PacketHistoryEntry> _packetHistory = new Queue<PacketHistoryEntry>(MaxPacketHistoryEntries);

    // ──────── Per-packet CSV logging (legacy WebRTC timing) ────────
    private string _csvPath;
    private StreamWriter _csvWriter;
    private readonly object _csvLock = new object();

    // ──────── UDP-style queue + packet logs (match IsaacUR3Client.cs) ────────
    private string _queueCsvLogPath = null;
    private string _packetCsvLogPath = null;

    // ──────── Application-level jitter (RFC 3550) for state channel ────────
    // Jitter tracks variability in inter-arrival time of robot_state packets.
    // D(i) = (recv_interval - send_interval) for consecutive packets
    // J(i) = J(i-1) + (|D(i)| - J(i-1)) / 16
    private long _prevPktSendTs = -1;    // sender timestamp (ms) of previous packet
    private double _prevPktRecvTime = 0; // local receive time (sec) of previous packet
    private float _stateJitterMs = 0f;   // smoothed jitter estimate (ms)

    // ──────── WebRTC Stats Logging (built-in GetStats API) ────────
    [Header("WebRTC Stats Logging")]
    [Tooltip("Interval in seconds between stats queries")]
    public float statsIntervalSec = 1.0f;

    private string _statsCsvPath;
    private StreamWriter _statsCsvWriter;
    private readonly object _statsCsvLock = new object();
    private Coroutine _statsCoroutine;

    // Per-channel delta tracking (keyed by channel label)
    private Dictionary<string, ulong> _prevBytesSent = new Dictionary<string, ulong>();
    private Dictionary<string, ulong> _prevBytesRecv = new Dictionary<string, ulong>();
    private Dictionary<string, uint> _prevMsgsSent  = new Dictionary<string, uint>();
    private Dictionary<string, uint> _prevMsgsRecv  = new Dictionary<string, uint>();

    // Transport-level delta tracking
    private ulong _prevTransportBytesSent = 0;
    private ulong _prevTransportBytesRecv = 0;

    // ================================================================
    // UNITY LIFECYCLE
    // ================================================================
    void Start()
    {
        // Reduce render-frame capping so Update/LateUpdate can run at higher frequency.
        // QualitySettings.vSyncCount = 0;
        // Application.targetFrameRate = 16;

        TryInitializeWebRTC();
        InitCsv();
        InitStatsCsv();
        ClearPacketHistory();
        if (ipInput != null) ipInput.text = signalingUrl;
        UpdateStatus("Ready (WebRTC)...", Color.white);

        // Wire disconnect button if assigned in Inspector
        if (disconnectButton != null)
        {
            disconnectButton.onClick.AddListener(Disconnect);
            disconnectButton.gameObject.SetActive(false); // hidden until connected
        }

        // CSV queue-size logger (one sample per frame, including queue size = 0)
        try
        {
            string timestamp = DateTime.UtcNow.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
            _queueCsvLogPath = Path.Combine(
                Application.persistentDataPath,
                $"received_queue_size_log_webrtc_{timestamp}.csv");
            File.WriteAllText(_queueCsvLogPath, "utc_iso,unix_ms,received_queue_size\n");
        }
        catch (Exception) { _queueCsvLogPath = null; }

        // CSV packet logger (one row per accepted packet)
        try
        {
            string timestamp = DateTime.UtcNow.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
            _packetCsvLogPath = Path.Combine(Application.persistentDataPath, $"unity_packet_log_webrtc_{timestamp}.csv");
            File.WriteAllText(
                _packetCsvLogPath,
                "utc_iso,unix_ms,seq,cmd_seq_echo,t0_echo,t1_echo,t3_recv,tq_dequeue,t4_physics,t5_send,t6_unity\n");
        }
        catch (Exception) { _packetCsvLogPath = null; }
    }

    void Update()
    {
        // NativeWebSocket requires dispatching messages on the main thread
#if !UNITY_WEBGL || UNITY_EDITOR
        _ws?.DispatchMessageQueue();
#endif

        // Snapshot queue size before draining so the log reflects real backlog.
        int queueSizeSnapshot = _packetQueue.Count;

        // ── Drain packet queue — apply joints from newest, discard older ──
        PendingPacket latest = default;
        bool hasPacket = false;
        while (_packetQueue.TryDequeue(out PendingPacket p))
        {
            latest = p;
            hasPacket = true;
        }

        // Log queue size every frame, including zero.
        TryAppendReceivedQueueCsv(queueSizeSnapshot);

        if (hasPacket)
        {
            var pkt = latest.data;

            // Sequence check (allow reset when server restarts)
            if (pkt.seq > _lastReceivedSeq || _lastReceivedSeq == -1)
            {
                _lastReceivedSeq = pkt.seq;
                LastPacketTimestamp = (pkt.t5_send > 0) ? pkt.t5_send : pkt.t6_unity;
                LastReceivedSeqPublic = pkt.seq;

                if (pkt.cmd_seq_echo > _lastConsumedCmdSeq && pkt.t1_echo > 0)
                {
                    _lastConsumedCmdSeq = pkt.cmd_seq_echo;
                    T1_ForLastResponse = pkt.t1_echo;
                    T0_InputCapture = pkt.t0_echo;
                }
                else
                {
                    T1_ForLastResponse = T1_PacketSent;
                }

                T6_UnityReceived = pkt.t6_unity;
                T3_IsaacReceived = pkt.t3_recv;
                TQ_IsaacDequeue = pkt.tq_dequeue;
                T4_PhysicsApplied = pkt.t4_physics;
                T5_IsaacSend = pkt.t5_send;
                LastReceivedCmdSeq = pkt.cmd_seq_echo;

                lock (StateLock)
                {
                    LatestState = pkt;
                }

                // Log one row per accepted packet (UDP-style)
                TryAppendPacketCsv(pkt);

                // ── RFC 3550 jitter ──
                double nowSec = GetUnixTimeSeconds();
                long sendTs = (pkt.t5_send > 0) ? pkt.t5_send : pkt.ts;
                if (_prevPktSendTs >= 0 && sendTs > 0)
                {
                    double recvIntervalMs = (nowSec - _prevPktRecvTime) * 1000.0;
                    double sendIntervalMs = (double)(sendTs - _prevPktSendTs);
                    double d = recvIntervalMs - sendIntervalMs;
                    _stateJitterMs += (float)((Math.Abs(d) - _stateJitterMs) / 16.0);
                    CurrentJitterMs = _stateJitterMs;
                }
                _prevPktSendTs = sendTs;
                _prevPktRecvTime = nowSec;

                // ── Apply joint angles ──
                for (int i = 0; i < 6; i++)
                {
                    JointAnglesRad[i] =
                        (pkt.state[i] * jointSigns[i])
                        + (visualOffsets[i] * Mathf.Deg2Rad);
                }

                // Stash for LateUpdate to stamp T7
                _pendingForLate = latest;
                _hasPendingLate = true;
            }

            // Debug first few packets
            if (_lastReceivedSeq < 5)
                Debug.Log($"[WebRTC] Received State seq={pkt.seq} t0_echo={pkt.t0_echo} t5={pkt.t5_send}");
        }
    }

    // ================================================================
    // LATE UPDATE — T7 = rendered (after all transforms applied this frame)
    // ================================================================
    void LateUpdate()
    {
        if (!_hasPendingLate) return;
        _hasPendingLate = false;

        var pending = _pendingForLate;
        long t7 = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();  // T7 = rendered (ms)
        float jitterSnapshot = _stateJitterMs;
        PacketHistoryEntry entry = BuildPacketHistoryEntry(pending.data, pending.t6, t7, jitterSnapshot);
        AppendPacketHistory(entry);

        // Log ALL state packets (no filtering)
        WriteCsvRow(entry);
    }

    void OnDestroy()
    {
        Cleanup();
        TryDisposeWebRTC();
    }

    void OnApplicationQuit()
    {
        Cleanup();
    }

    // ================================================================
    // PUBLIC — Called from your UI Connect button
    // ================================================================
    public void Connect()
    {
        // Reset sequence number in case the server was restarted
        _lastReceivedSeq = -1;
        _lastConsumedCmdSeq = -1;

        string url = (ipInput != null && !string.IsNullOrEmpty(ipInput.text))
            ? ipInput.text.Trim()
            : signalingUrl;

        // Allow user to type just an IP — we build the ws:// URL
        if (!url.StartsWith("ws://") && !url.StartsWith("wss://"))
            url = $"ws://{url}:8765";

        signalingUrl = url;
        UpdateStatus("Connecting...", Color.yellow);
        StartCoroutine(ConnectCoroutine());
    }

    // ================================================================
    // PUBLIC — Called from your UI Disconnect button (or programmatically)
    // ================================================================
    /// <summary>
    /// Gracefully tears down the WebRTC session:
    ///   1. Sends a "bye" on the signaling WebSocket so the server knows immediately.
    ///   2. Closes DataChannels, PeerConnection, and the signaling WebSocket.
    ///   3. Flushes & closes CSV files.
    ///   4. Shows the connection panel again so the user can reconnect.
    /// You can wire this to an on-screen Button's onClick event in the Inspector.
    /// </summary>
    public void Disconnect()
    {
        Debug.Log("[WebRTC] 🔌 Disconnect requested by user");

        // Send a "bye" to the signaling server so the Isaac side is notified
        // immediately (instead of waiting for the ICE timeout).
        if (_ws != null && _ws.State == WebSocketState.Open)
        {
            try { _ws.SendText("{\"bye\": true}"); } catch { }
        }

        // Tear everything down
        Cleanup();

        // Reset joint angles to zero (prevents stale pose)
        for (int i = 0; i < 6; i++) JointAnglesRad[i] = 0f;
        _lastReceivedSeq = -1;

        // Re-show connection panel so the user can reconnect
        if (connectionPanel != null)
            connectionPanel.SetActive(true);

        // Hide disconnect button
        if (disconnectButton != null)
            disconnectButton.gameObject.SetActive(false);

        UpdateStatus("Disconnected — tap Connect to rejoin", Color.white);
    }

    /// <summary>
    /// Returns a snapshot of the most recent packet history entries. Copying ensures
    /// callers can iterate without holding internal locks.
    /// </summary>
    public static PacketHistoryEntry[] GetPacketHistorySnapshot()
    {
        lock (_packetHistoryLock)
        {
            return _packetHistory.ToArray();
        }
    }

    /// <summary>
    /// Clears the retained packet history. Invoked automatically on Disconnect/Cleanup
    /// but exposed publicly so external tooling can reset statistics if desired.
    /// </summary>
    public static void ClearPacketHistory()
    {
        lock (_packetHistoryLock)
        {
            _packetHistory.Clear();
        }
    }

    // ================================================================
    // PUBLIC — Send a joystick delta (same signature as IsaacUR3Logging.cs)
    // T0 = input capture time (passed from button_check), T1 = send time
    // ================================================================
    public void SendDelta(int index, float amount, long t0 = 0)
    {
        if (_dcCommands != null && _dcCommands.ReadyState == RTCDataChannelState.Open)
        {
            long t1 = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            if (t0 == 0) t0 = t1;

            T0_InputCapture = t0;
            T1_PacketSent = t1;

            int seq = Interlocked.Increment(ref _cmdSeq);

            // --- Drift Analytics Tracking ---
            if (!ExpectedInitialized)
            {
                for (int i = 0; i < 6; i++) ExpectedJointAngles[i] = JointAnglesRad[i];
                ExpectedInitialized = true;
            }
            // Scale the expected movement by the joint sign to match how incoming states are parsed
            ExpectedJointAngles[index] += (amount * jointSigns[index]);
            float[] snapshot = new float[6];
            Array.Copy(ExpectedJointAngles, snapshot, 6);
            CmdToExpected[seq] = snapshot;

            string json = string.Format(CultureInfo.InvariantCulture,
                "{{\"delta\":[{0},{1}],\"t0\":{2},\"t1\":{3},\"cmd_seq\":{4}}}",
                index, amount, t0, t1, seq);
            _dcCommands.Send(json);
        }
    }

    // --- CONTINUOUS SEND (16 ms) ---
    // Hook this to a UI Button OnClick to start sending continuously.
    public void StartContinuousDelta(int index, float amount)
    {
        _continuousIndex = index;
        _continuousAmount = amount;
        _continuousSendActive = true;

        if (_continuousSendRoutine == null)
        {
            _continuousSendRoutine = StartCoroutine(ContinuousSendLoop());
        }
    }

    // Hook this to a UI Button (or OnPointerUp) to stop continuous sending.
    public void StopContinuousDelta()
    {
        _continuousSendActive = false;
        if (_continuousSendRoutine != null)
        {
            StopCoroutine(_continuousSendRoutine);
            _continuousSendRoutine = null;
        }
    }

    private IEnumerator ContinuousSendLoop()
    {
       
        while (_continuousSendActive)
        {
            long t0 = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            SendDelta(_continuousIndex, _continuousAmount, t0);
            yield return null;
        }
        _continuousSendRoutine = null;
    }

    // ================================================================
    // COROUTINE — drives the async signaling flow on the main thread
    // ================================================================
    private IEnumerator ConnectCoroutine()
    {
        // ---------- 1. WebSocket to signaling server ----------
        _ws = new WebSocket(signalingUrl);

        _ws.OnOpen += () =>
        {
            Debug.Log("[Signaling] WebSocket opened");
            // Register as "unity"
            _ws.SendText("{\"register\": \"unity\"}");
        };

        _ws.OnMessage += (bytes) =>
        {
            string raw = Encoding.UTF8.GetString(bytes);
            OnSignalingMessage(raw);
        };

        _ws.OnError += (err) =>
        {
            Debug.LogError($"[Signaling] WS error: {err}");
            UpdateStatus("Signaling Error", Color.red);
        };

        _ws.OnClose += (code) =>
        {
            Debug.Log("[Signaling] WS closed");
        };

        _ws.Connect();

        // Wait until signaling WebSocket is open
        float timer = 0f;
        while (_ws.State != WebSocketState.Open && timer < 10f)
        {
            timer += Time.deltaTime;
            yield return null;
        }

        if (_ws.State != WebSocketState.Open)
        {
            UpdateStatus("Signaling timeout", Color.red);
            yield break;
        }

        Debug.Log("[Signaling] Connected to signaling server");
        UpdateStatus("Signaling OK – waiting for offer...", Color.yellow);
    }

    // ================================================================
    // SIGNALING MESSAGE HANDLER (runs on main thread via DispatchMessageQueue)
    // ================================================================
    private void OnSignalingMessage(string raw)
    {
        try
        {
            var msg = JsonUtility.FromJson<SignalingMsg>(raw);

            // Registration ACK
            if (!string.IsNullOrEmpty(msg.registered))
            {
                Debug.Log($"[Signaling] Registered as: {msg.registered}");
                return;
            }

            // SDP Offer from Isaac
            if (!string.IsNullOrEmpty(msg.sdp) && msg.type == "offer")
            {
                Debug.Log("[Signaling] 📥 Received SDP Offer");
                StartCoroutine(HandleOffer(msg.sdp));
                return;
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[Signaling] Parse error: {e.Message}  raw={raw}");
        }
    }

    // ================================================================
    // HANDLE SDP OFFER → create PeerConnection → send Answer
    // ================================================================
    private IEnumerator HandleOffer(string sdp)
    {
        // ---------- PeerConnection ----------
        var config = new RTCConfiguration
        {
            iceServers = new[]
            {
                new RTCIceServer { urls = new[] { "stun:stun.l.google.com:19302" } }
            }
        };

        _pc = new RTCPeerConnection(ref config);

        _pc.OnDataChannel = channel =>
        {
            if (channel.Label == "commands")
            {
                // Unused locally, server shouldn't be creating it anyway
            }
            if (channel.Label == "robot_state")
            {
                _dcState = channel;
                _dcState.OnMessage = bytes =>
                {
                    // T6 captured immediately on WebRTC background thread
                    long t6 = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
                    string json = Encoding.UTF8.GetString(bytes);

                    try
                    {
                        var packet = JsonUtility.FromJson<IsaacState>(json);
                        packet.t6_unity = t6; // Apply T6 immediately in background thread!

                        _packetQueue.Enqueue(new PendingPacket
                        {
                            data = packet,
                            t6 = t6
                        });
                    }
                    catch { }
                };
            }
        };

        _pc.OnIceCandidate = candidate =>
        {
            // We wait for gathering complete, so nothing to trickle
        };

        _pc.OnIceConnectionChange = state =>
        {
            Debug.Log($"[WebRTC] ICE state: {state}");
            if (state == RTCIceConnectionState.Connected)
            {
                _isConnected = true;
                UpdateStatus("WebRTC Connected!", Color.green);
                Invoke("HidePanel", 1.0f);

                // Show disconnect button
                if (disconnectButton != null)
                    disconnectButton.gameObject.SetActive(true);

                // Start periodic WebRTC stats collection
                if (_statsCoroutine == null)
                    _statsCoroutine = StartCoroutine(CollectWebRTCStats());
            }
            else if (state == RTCIceConnectionState.Disconnected ||
                     state == RTCIceConnectionState.Failed)
            {
                _isConnected = false;
                UpdateStatus("Disconnected", Color.red);

                // Hide disconnect button
                if (disconnectButton != null)
                    disconnectButton.gameObject.SetActive(false);

                // Stop stats collection
                if (_statsCoroutine != null)
                {
                    StopCoroutine(_statsCoroutine);
                    _statsCoroutine = null;
                }
            }
        };

        // ---------- Handle incoming DataChannels from Isaac ----------
        _pc.OnDataChannel = channel =>
        {
            Debug.Log($"[WebRTC] 📥 Remote DataChannel: {channel.Label}");

            if (channel.Label == "robot_state")
            {
                _dcState = channel; // MUST store reference to prevent garbage collection!
                
                // Receive-only from Isaac — stamp T6 immediately, enqueue for Update()
                _dcState.OnMessage = (msgBytes) =>
                {
                    long t6 = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();  // T6 = Unity receive time (ms)
                    string json = Encoding.UTF8.GetString(msgBytes);
                    try
                    {
                        IsaacState pkt = JsonUtility.FromJson<IsaacState>(json);
                        if (pkt != null && pkt.state != null && pkt.state.Length == 6)
                        {
                            pkt.t6_unity = t6;
                            _packetQueue.Enqueue(new PendingPacket { data = pkt, t6 = t6 });
                        }
                    }
                    catch (Exception e)
                    {
                        Debug.LogWarning($"[WebRTC] State parse error: {e.Message}");
                    }
                };
                
                _dcState.OnClose = () => 
                {
                    Debug.LogWarning("[WebRTC] 🔌 Remote DataChannel 'robot_state' CLOSED");
                };
            }
        };

        // ---------- Create our own DataChannel for sending commands ----------
        // Send-only from Unity to Isaac
         var dcInit = new RTCDataChannelInit { ordered = true, maxRetransmits = null };
        // var dcInit = new RTCDataChannelInit { ordered = false, maxRetransmits = 0 };
        _dcCommands = _pc.CreateDataChannel("commands", dcInit);
        _dcCommands.OnOpen = () => Debug.Log("[WebRTC] ✅ DataChannel 'commands' OPEN (Send-only)");

        // ---------- Set Remote Description (the Offer) ----------
        var offerDesc = new RTCSessionDescription { sdp = sdp, type = RTCSdpType.Offer };
        var setRemoteOp = _pc.SetRemoteDescription(ref offerDesc);
        yield return setRemoteOp;

        if (setRemoteOp.IsError)
        {
            Debug.LogError($"[WebRTC] SetRemoteDescription failed: {setRemoteOp.Error.message}");
            UpdateStatus("SDP Error", Color.red);
            yield break;
        }

        // ---------- Create Answer ----------
        var answerOp = _pc.CreateAnswer();
        yield return answerOp;

        if (answerOp.IsError)
        {
            Debug.LogError($"[WebRTC] CreateAnswer failed: {answerOp.Error.message}");
            yield break;
        }

        var answerDesc = answerOp.Desc;
        var setLocalOp = _pc.SetLocalDescription(ref answerDesc);
        yield return setLocalOp;

        // Wait for ICE gathering to finish so candidates are baked into SDP
        while (_pc.GatheringState != RTCIceGatheringState.Complete)
            yield return null;

        // ---------- Send Answer back via signaling ----------
        string answerJson = JsonUtility.ToJson(new SignalingMsg
        {
            sdp = _pc.LocalDescription.sdp,
            type = "answer"
        });

        _ws.SendText(answerJson);
        Debug.Log("[Signaling] 📤 Answer sent");
        UpdateStatus("Answer sent – connecting...", Color.yellow);
    }

    // ================================================================
    // PER-PACKET CSV LOGGING (same pattern as IsaacUR3Logging.cs)
    // ================================================================
    private void InitCsv()
    {
        try
        {
            string dir = Application.persistentDataPath;
            Debug.Log($"[Timing] 📂 persistentDataPath = {dir}");
            string ts = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            _csvPath = Path.Combine(dir, $"webrtc_timing_{ts}.csv");

            _csvWriter = new StreamWriter(_csvPath, append: false, encoding: Encoding.UTF8);
            _csvWriter.WriteLine(
                "timestamp_utc," +
                "seq,cmd_seq_echo,has_cmd,server_ts_ms," +
                "T0_input_ns,T1_send_ns,T3_isaac_recv_ns,T4_physics_ns,T5_isaac_send_ns," +
                "T6_unity_recv_ns,T7_rendered_ns," +
                "RTT_ms,Processing_ms,Network_est_ms,RenderDelay_ms,MTP_ms," +
                "Jitter_ms," +
                "NOTE_clocks"
            );
            _csvWriter.Flush();
            Debug.Log($"[Timing] ✅ CSV log created: {_csvPath}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[Timing] ❌ Failed to create CSV: {e.Message}");
        }
    }
    
    private void CloseCsv()
    {
        lock (_csvLock)
        {
            if (_csvWriter != null)
            {
                try { _csvWriter.Flush(); _csvWriter.Close(); } catch { }
                _csvWriter = null;
                Debug.Log($"[Timing] 📊 CSV log closed: {_csvPath}");
            }
        }
    }



    /// <summary>
    /// Write one CSV row per rendered state packet.
    /// T0..T1, T6, T7 are on the Unity clock (ns).
    /// T3..T5 are on the Isaac clock (ns).
    /// Jitter is RFC 3550 smoothed estimate.
    /// </summary>
    private PacketHistoryEntry BuildPacketHistoryEntry(IsaacState pkt, long t6_ms, long t7_ms, float jitterMsSnapshot)
    {
        long t0_ns = pkt.t0_echo > 0 ? pkt.t0_echo * 1_000_000L : 0;
        long t1_ns = pkt.t1_echo > 0 ? pkt.t1_echo * 1_000_000L : 0;
        long t6_ns = t6_ms * 1_000_000L;
        long t7_ns = t7_ms * 1_000_000L;

        // RTT = T6 - T1  (Unity clock round-trip: send → receive)
        double rtt_ms         = pkt.t1_echo > 0 ? (t6_ms  - pkt.t1_echo) : double.NaN;
        // Processing = T4 - T3  (Isaac clock: receive → physics done)
        double processing_ms  = pkt.t3_recv > 0 ? (pkt.t4_physics - pkt.t3_recv) : double.NaN;
        // Network ≈ (RTT - Processing) / 2  — approximate (cross-clock)
        double network_ms     = (double.IsNaN(rtt_ms) || double.IsNaN(processing_ms))
                                ? double.NaN
                                : (rtt_ms - processing_ms) / 2.0;
        // RenderDelay = T7 - T6  (Unity clock: receive → rendered)
        double render_ms      = (t7_ms - t6_ms);
        // MTP = T7 - T0  (Unity clock: input → rendered)
        double mtp_ms         = pkt.t0_echo > 0 ? (t7_ms - pkt.t0_echo) : double.NaN;

        return new PacketHistoryEntry
        {
            TimestampUtc = DateTime.UtcNow,
            Seq = pkt.seq,
            CmdSeqEcho = pkt.cmd_seq_echo,
            HasCommand = pkt.has_cmd,
            ServerTimestampMs = (pkt.t5_send > 0) ? pkt.t5_send : pkt.ts,
            T0Ns = t0_ns,
            T1Ns = t1_ns,
            T3Ns = pkt.t3_recv * 1_000_000L,
            T4Ns = pkt.t4_physics * 1_000_000L,
            T5Ns = pkt.t5_send * 1_000_000L,
            T6Ns = t6_ns,
            T7Ns = t7_ns,
            RttMs = rtt_ms,
            ProcessingMs = processing_ms,
            NetworkEstMs = network_ms,
            RenderDelayMs = render_ms,
            MotionToPhotonMs = mtp_ms,
            JitterMs = jitterMsSnapshot 
        };
    }

    private static void AppendPacketHistory(PacketHistoryEntry entry)
    {
        lock (_packetHistoryLock)
        {
            if (_packetHistory.Count >= MaxPacketHistoryEntries)
                _packetHistory.Dequeue();

            _packetHistory.Enqueue(entry);
        }
    }

    private void WriteCsvRow(PacketHistoryEntry entry)
    {
        string F(double v) => double.IsNaN(v) ? "N/A" : v.ToString("F4", CultureInfo.InvariantCulture);

        lock (_csvLock)
        {
            if (_csvWriter == null) return;
            try
            {
                _csvWriter.WriteLine(
                    $"{entry.TimestampUtc:O}," +
                    $"{entry.Seq},{entry.CmdSeqEcho},{(entry.HasCommand ? 1 : 0)},{entry.ServerTimestampMs}," +
                    $"{entry.T0Ns},{entry.T1Ns},{entry.T3Ns},{entry.T4Ns},{entry.T5Ns}," +
                    $"{entry.T6Ns},{entry.T7Ns}," +
                    $"{F(entry.RttMs)},{F(entry.ProcessingMs)},{F(entry.NetworkEstMs)},{F(entry.RenderDelayMs)},{F(entry.MotionToPhotonMs)}," +
                    $"{entry.JitterMs.ToString("F2", CultureInfo.InvariantCulture)}," +
                    "T3-T5 on Isaac clock; Network_est approximate"
                );
                _csvWriter.Flush();
            }
            catch (Exception e)
            {
                Debug.LogError($"[Timing] ❌ CSV write error: {e.Message}");
            }
        }
    }

    private void TryAppendReceivedQueueCsv(int queueSize)
    {
        if (string.IsNullOrEmpty(_queueCsvLogPath)) return;
        try
        {
            DateTime utcNow = DateTime.UtcNow;
            long unixMs = new DateTimeOffset(utcNow).ToUnixTimeMilliseconds();
            string row = string.Format(CultureInfo.InvariantCulture,
                "{0},{1},{2}\n",
                utcNow.ToString("O", CultureInfo.InvariantCulture),
                unixMs,
                queueSize);
            File.AppendAllText(_queueCsvLogPath, row);
        }
        catch (Exception) { }
    }

    private void TryAppendPacketCsv(IsaacState packet)
    {
        if (string.IsNullOrEmpty(_packetCsvLogPath) || packet == null) return;
        try
        {
            DateTime utcNow = DateTime.UtcNow;
            long unixMs = new DateTimeOffset(utcNow).ToUnixTimeMilliseconds();
            string row = string.Format(
                CultureInfo.InvariantCulture,
                "{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10}\n",
                utcNow.ToString("O", CultureInfo.InvariantCulture),
                unixMs,
                packet.seq,
                packet.cmd_seq_echo,
                packet.t0_echo,
                packet.t1_echo,
                packet.t3_recv,
                packet.tq_dequeue,
                packet.t4_physics,
                packet.t5_send,
                packet.t6_unity);
            File.AppendAllText(_packetCsvLogPath, row);
        }
        catch (Exception) { }
    }

    // ================================================================
    // WEBRTC STATS LOGGING (uses built-in RTCPeerConnection.GetStats)
    // ================================================================

    /// <summary>
    /// Initialise a separate CSV file for periodic WebRTC channel/transport stats.
    /// Columns: timestamp_utc, channel_label, state,
    ///          bytesSent, bytesReceived, messagesSent, messagesReceived,
    ///          delta_bytesSent, delta_bytesReceived, delta_msgsSent, delta_msgsRecv,
    ///          throughput_send_kbps, throughput_recv_kbps,
    ///          transport_bytesSent, transport_bytesRecv,
    ///          transport_throughput_send_kbps, transport_throughput_recv_kbps,
    ///          ice_rtt_ms, ice_availableOutBitrate_kbps
    /// </summary>
    private void InitStatsCsv()
    {
        try
        {
            string dir = Application.persistentDataPath;
            string ts = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            _statsCsvPath = Path.Combine(dir, $"webrtc_channel_stats_{ts}.csv");

            _statsCsvWriter = new StreamWriter(_statsCsvPath, append: false, encoding: Encoding.UTF8);
            _statsCsvWriter.WriteLine(
                "timestamp_utc," +
                "channel_label,channel_state," +
                "bytesSent,bytesReceived,messagesSent,messagesReceived," +
                "delta_bytesSent,delta_bytesReceived,delta_msgsSent,delta_msgsRecv," +
                "throughput_send_kbps,throughput_recv_kbps," +
                "transport_bytesSent,transport_bytesRecv," +
                "transport_throughput_send_kbps,transport_throughput_recv_kbps," +
                "ice_rtt_ms,ice_availableOutBitrate_kbps"
            );
            _statsCsvWriter.Flush();
            Debug.Log($"[Stats] ✅ Channel stats CSV created: {_statsCsvPath}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[Stats] ❌ Failed to create stats CSV: {e.Message}");
        }
    }

    private void CloseStatsCsv()
    {
        lock (_statsCsvLock)
        {
            if (_statsCsvWriter != null)
            {
                try { _statsCsvWriter.Flush(); _statsCsvWriter.Close(); } catch { }
                _statsCsvWriter = null;
                Debug.Log($"[Stats] 📊 Channel stats CSV closed: {_statsCsvPath}");
            }
        }
    }

    /// <summary>
    /// Coroutine that periodically calls RTCPeerConnection.GetStats() and
    /// extracts per-DataChannel stats, transport stats, and ICE candidate-pair
    /// stats. Computes deltas and throughput (kbps) between samples.
    /// </summary>
    private IEnumerator CollectWebRTCStats()
    {
        Debug.Log($"[Stats] 🔄 Stats collection started (interval={statsIntervalSec}s)");

        while (_isConnected && _pc != null)
        {
            yield return new WaitForSeconds(statsIntervalSec);

            if (_pc == null) break;

            // ── 1. Call GetStats (async operation) ──
            var op = _pc.GetStats();
            yield return op;

            if (op.IsError)
            {
                Debug.LogWarning("[Stats] GetStats failed");
                continue;
            }

            RTCStatsReport report = op.Value;
            string nowStr = DateTime.UtcNow.ToString("O");
            float interval = statsIntervalSec; // seconds between samples

            // ── Temporaries for transport & ICE (filled once per report) ──
            ulong transportBytesSent = 0;
            ulong transportBytesRecv = 0;
            double iceRttMs = -1;
            double iceAvailOutKbps = -1;
            bool hasTransport = false;

            // ── 2. Walk all stats objects ──
            foreach (var kv in report.Stats)
            {
                RTCStats stat = kv.Value;

                // ── ICE candidate-pair stats (RTT, available bitrate) ──
                if (stat is RTCIceCandidatePairStats icePair)
                {
                    // Use the nominated pair if available (state == "succeeded")
                    if (icePair.state == "succeeded" || icePair.nominated)
                    {
                        iceRttMs = icePair.currentRoundTripTime * 1000.0; // sec → ms
                        iceAvailOutKbps = icePair.availableOutgoingBitrate / 1000.0; // bps → kbps
                    }
                }

                // ── Transport-level stats ──
                if (stat is RTCTransportStats ts)
                {
                    transportBytesSent = ts.bytesSent;
                    transportBytesRecv = ts.bytesReceived;
                    hasTransport = true;
                }

                // ── DataChannel stats ──
                if (stat is RTCDataChannelStats dc)
                {
                    string label = dc.label ?? "unknown";
                    string state = dc.state ?? "unknown";
                    ulong bSent = dc.bytesSent;
                    ulong bRecv = dc.bytesReceived;
                    uint mSent = dc.messagesSent;
                    uint mRecv = dc.messagesReceived;

                    // Compute deltas
                    ulong prevBS = _prevBytesSent.ContainsKey(label) ? _prevBytesSent[label] : 0;
                    ulong prevBR = _prevBytesRecv.ContainsKey(label) ? _prevBytesRecv[label] : 0;
                    uint  prevMS = _prevMsgsSent.ContainsKey(label) ? _prevMsgsSent[label] : 0;
                    uint  prevMR = _prevMsgsRecv.ContainsKey(label) ? _prevMsgsRecv[label] : 0;

                    ulong dBS = bSent - prevBS;
                    ulong dBR = bRecv - prevBR;
                    uint  dMS = mSent - prevMS;
                    uint  dMR = mRecv - prevMR;

                    // Throughput in kbps  (kilobits per second)
                    double tpSend = (dBS * 8.0 / 1000.0) / interval;
                    double tpRecv = (dBR * 8.0 / 1000.0) / interval;

                    // Store for next delta
                    _prevBytesSent[label] = bSent;
                    _prevBytesRecv[label] = bRecv;
                    _prevMsgsSent[label]  = mSent;
                    _prevMsgsRecv[label]  = mRecv;

                    // Console log (every sample)
                    Debug.Log($"[Stats] 📊 DC '{label}' [{state}]  " +
                              $"sent={bSent}B ({mSent}msg)  recv={bRecv}B ({mRecv}msg)  " +
                              $"Δsent={dBS}B Δrecv={dBR}B  " +
                              $"tp_send={tpSend:F1}kbps tp_recv={tpRecv:F1}kbps");

                    // ── Transport deltas ──
                    ulong tDBS = 0, tDBR = 0;
                    double ttSend = 0, ttRecv = 0;
                    if (hasTransport)
                    {
                        tDBS = transportBytesSent - _prevTransportBytesSent;
                        tDBR = transportBytesRecv - _prevTransportBytesRecv;
                        ttSend = (tDBS * 8.0 / 1000.0) / interval;
                        ttRecv = (tDBR * 8.0 / 1000.0) / interval;
                    }

                    // ── Write CSV row ──
                    lock (_statsCsvLock)
                    {
                        if (_statsCsvWriter != null)
                        {
                            try
                            {
                                string F(double v) => v < 0 ? "N/A" : v.ToString("F2", CultureInfo.InvariantCulture);
                                _statsCsvWriter.WriteLine(
                                    $"{nowStr}," +
                                    $"{label},{state}," +
                                    $"{bSent},{bRecv},{mSent},{mRecv}," +
                                    $"{dBS},{dBR},{dMS},{dMR}," +
                                    $"{tpSend.ToString("F2", CultureInfo.InvariantCulture)}," +
                                    $"{tpRecv.ToString("F2", CultureInfo.InvariantCulture)}," +
                                    $"{transportBytesSent},{transportBytesRecv}," +
                                    $"{(hasTransport ? ttSend.ToString("F2", CultureInfo.InvariantCulture) : "N/A")}," +
                                    $"{(hasTransport ? ttRecv.ToString("F2", CultureInfo.InvariantCulture) : "N/A")}," +
                                    $"{F(iceRttMs)},{F(iceAvailOutKbps)}"
                                );
                                _statsCsvWriter.Flush();
                            }
                            catch (Exception e)
                            {
                                Debug.LogError($"[Stats] CSV write error: {e.Message}");
                            }
                        }
                    }
                }
            } // end foreach stats

            // Update transport-level previous values after all channels processed
            if (hasTransport)
            {
                _prevTransportBytesSent = transportBytesSent;
                _prevTransportBytesRecv = transportBytesRecv;
            }

            // Log summary ICE info
            if (iceRttMs >= 0)
            {
                Debug.Log($"[Stats] 🌐 ICE RTT={iceRttMs:F2}ms  AvailOut={iceAvailOutKbps:F1}kbps  " +
                          $"Transport sent={transportBytesSent}B recv={transportBytesRecv}B");
            }

            report.Dispose();
        }

        Debug.Log("[Stats] ⏹ Stats collection stopped");
    }

    // ================================================================
    // HELPERS
    // ================================================================
    private double GetUnixTimeSeconds()
    {
        return (DateTime.UtcNow - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).TotalSeconds;
    }

    private void Cleanup()
    {
        _isConnected = false;

        // Stop stats collection coroutine
        if (_statsCoroutine != null)
        {
            StopCoroutine(_statsCoroutine);
            _statsCoroutine = null;
        }

        CloseCsv();
        CloseStatsCsv();
    ClearPacketHistory();

        if (_ws != null)
        {
            try { _ws.Close(); } catch { }
            _ws = null;
        }

        if (_dcCommands != null)
        {
            _dcCommands.Close();
            _dcCommands = null;
        }

        if (_dcState != null)
        {
            _dcState.Close();
            _dcState = null;
        }

        if (_pc != null)
        {
            _pc.Close();
            _pc.Dispose();
            _pc = null;
        }
    }

    private void UpdateStatus(string message, Color color)
    {
        if (statusText != null)
        {
            statusText.text = message;
            statusText.color = color;
        }
    }

    private void HidePanel()
    {
        if (connectionPanel != null) connectionPanel.SetActive(false);
    }

    // Small JSON helper for signaling messages
    [Serializable]
    private class SignalingMsg
    {
        public string registered;
        public string sdp;
        public string type;
    }

    // Reflection helpers: some versions of the Unity WebRTC package expose
    // WebRTC.Initialize/Dispose as static methods, others do not. To avoid
    // hard compile dependencies on those symbols we call them via reflection
    // if present.
    private void TryInitializeWebRTC()
    {
        var t = FindWebRTCType();
        if (t == null) return;
        var m = t.GetMethod("Initialize", Type.EmptyTypes);
        try { m?.Invoke(null, null); } catch { }
    }

    private void TryDisposeWebRTC()
    {
        var t = FindWebRTCType();
        if (t == null) return;
        var m = t.GetMethod("Dispose", Type.EmptyTypes);
        try { m?.Invoke(null, null); } catch { }
    }

    private Type FindWebRTCType()
    {
        // Try to locate Unity.WebRTC.WebRTC type across loaded assemblies.
        foreach (var asm in AppDomain.CurrentDomain.GetAssemblies())
        {
            try
            {
                var t = asm.GetType("Unity.WebRTC.WebRTC");
                if (t != null) return t;
            }
            catch { }
        }
        return null;
    }
}
