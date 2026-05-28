using UnityEngine;
using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Globalization;
using TMPro;
using UnityEngine.UI;

public class IsaacUR3Client : MonoBehaviour
{
    // --- SHARED DATA ---
    // Your link scripts read this array.
    public static float[] JointAnglesRad = new float[6];

    // ── Latency timestamps ───────────────────────────────────────────────────
    // Unity-side: UTC milliseconds via DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()
    public static long T0_InputCapture  = 0; // T0: input button captured         [Unity UTC ms]
    public static long T1_PacketSent    = 0; // T1: UDP packet left Unity          [Unity UTC ms]
    public static long T6_UnityReceived = 0; // T6: UDP packet arrived at Unity    [Unity UTC ms]
    // Isaac-side: UTC milliseconds from system clock (chrony-disciplined on Linux)
    public static long T3_IsaacReceived  = 0; // T3: Isaac recvfrom() timestamp    [Isaac UTC ms]
    public static long TQ_IsaacDequeue   = 0; // TQ: Isaac queue dequeue timestamp  [Isaac UTC ms]
    public static long T4_PhysicsApplied = 0; // T4: Isaac physics applied          [Isaac UTC ms]
    public static long T5_IsaacSend      = 0; // T5: Isaac sendto() timestamp       [Isaac UTC ms]

    // T1 that corresponds to the SAME send-receive cycle as the latest T6.
    // Set when a response arrives: captures whatever T1 was in-flight at that moment.
    // LatencyLogger uses this for RTT = T6 - T1_ForLastResponse (valid pair).
    public static long T1_ForLastResponse = 0;

    public static int LastReceivedSeqPublic = -1; // seq of the latest accepted packet
    public static int LastReceivedCmdSeq    = -1; // cmd_seq echoed back by Isaac (for pairing)
    public static float CurrentJitterMs     = 0f; // RFC 3550-style inter-arrival jitter

    // ── Published state snapshot (thread-safe) ───────────────────────────────
    // LatencyLogger reads this snapshot instead of individual statics, so all
    // fields (t0_echo, t1_echo, t3, t4, t5, t6, seq, cmd_seq_echo) are always
    // from the SAME packet — no torn reads, no mismatched pairs.
    public static readonly object StateLock = new object();
    public static IsaacState LatestState = null; // written in Update() (main thread) under StateLock
    public static ConcurrentDictionary<int, float[]> CmdToExpected = new ConcurrentDictionary<int, float[]>();

    private static float[] _expectedAnglesRad = new float[6];
    private static bool _expectedInitialized = false;
    private static readonly object _expectedLock = new object();

    // ---- Connection state events (subscribed by main_ui_control) ----
    public static event System.Action OnConnected;
    public static event System.Action OnDisconnected;

    [Header("UI Assignments")]
    public TMP_InputField ipInput;        
    public TMP_Text statusText;           
    public TMP_Text debugText;           // optional on-screen debug output
    public GameObject connectionPanel;    
    public RawImage videoDisplay;          // assign this to the VideoPanel RawImage
    public int videoListenPort = 11022;    // port to receive chunked JPEG frames
    public bool enableVideo = false;        // toggle video receiver on/off (Inspector)

    [Header("UDP Settings")]
    public string defaultIP = "192.168.1.228"; 
    public int sendPort = 11020;    
    public int listenPort = 11021;  

    [Header("Calibration")]
    public float[] visualOffsets = new float[6]; 
    public float[] jointSigns = new float[] { 1, 1, 1, 1, 1, 1 };

    // --- INTERNAL UDP VARS ---
    [Serializable] 
    public class IsaacState 
    { 
        public int   seq; 
        public int   cmd_seq_echo; // echoed Unity cmd_seq — used for exact T1 pairing
        public long  t0_echo;      // echoed Unity T0 UTC ms — input capture time
        public long  t1_echo;      // echoed Unity T1 UTC ms — used for exact RTT
        public long  t3_recv;      // T3: Isaac recvfrom() — UTC ms
        public long  tq_dequeue;   // TQ: Isaac queue dequeue — UTC ms
        public long  t4_physics;   // T4: Isaac physics applied — UTC ms
        public long  t5_send;      // T5: Isaac sendto() — UTC ms
        public long  t6_unity;     // T6: Unity recvfrom() — UTC ms (filled by ReceiveLoop, not Isaac)
        public float[] state; 
    }

    private TcpClient _cmdClient;
    private NetworkStream _cmdStream;
    private readonly object _cmdSendLock = new object();
    private UdpClient _udpReceiver;
    private IPEndPoint _serverEndPoint;
    private Thread _receiveThread;
    private UdpClient _videoReceiver;
    private Thread _videoThread;
    private bool _isRunning = false;
    private int _lastReceivedSeq = -1;
    private int _cmdSeq = 0;              // per-command sequence counter (Unity side)
    private int _lastConsumedCmdSeq = -1; // last cmd_seq whose echo was used for logging
    private Coroutine _continuousSendRoutine = null;
    private bool _continuousSendActive = false;
    private int _continuousIndex = 0;
    private float _continuousAmount = 0f;

    // Queue for passing received JSON from background thread to main thread
    private Queue<string> _receivedQueue = new Queue<string>();
    private object _queueLock = new object();
    // T6 paired with the latest enqueued JSON (written by receive thread, read in Update)
    private long _pendingT6 = 0;
    // Simple debug log queue so background threads can push messages for main thread display
    private Queue<string> _debugQueue = new Queue<string>();
    private object _debugLock = new object();
    private string _persistentLogPath = null;
    private string _queueCsvLogPath = null;
    private string _packetCsvLogPath = null;
    // Video-specific queues and buffers
    private Queue<byte[]> _videoQueue = new Queue<byte[]>();
    private object _videoLock = new object();

    // ── Application-level jitter (RFC 3550) for state packets ──────────────
    // D(i) = recv_interval - send_interval
    // J(i) = J(i-1) + (|D(i)| - J(i-1)) / 16
    private long _prevPktSendTs = -1;    // sender timestamp (ms) of previous packet
    private double _prevPktRecvTime = 0; // local receive time (sec) of previous packet
    private float _stateJitterMs = 0f;   // smoothed jitter estimate (ms)

    // Partial-frame assembly structure (keyed by frame id)
    private class VideoFrameBuffer
    {
        public int totalChunks;
        public Dictionary<int, byte[]> chunks = new Dictionary<int, byte[]>();
        public double firstSeenTs;
    }
    private Dictionary<uint, VideoFrameBuffer> _videoFrames = new Dictionary<uint, VideoFrameBuffer>();
    private object _videoFramesLock = new object();
    private int _maxVideoQueue = 3; // cap queued decoded frames
    private Texture2D _videoTexture = null;
    // Guard: ensure Unity's graphics device is ready before creating textures
    private bool _graphicsReady = false;

    void Start()
    {

        if (ipInput != null) ipInput.text = defaultIP; 
        UpdateStatus("Ready (TCP cmd + UDP state)...", Color.white);
        // prepare persistent log path
        try
        {
            _persistentLogPath = System.IO.Path.Combine(Application.persistentDataPath, "isaac_client_log.txt");
            System.IO.File.AppendAllText(_persistentLogPath, $"\n--- Start: {DateTime.UtcNow:O} ---\n");
        }
        catch (Exception) { _persistentLogPath = null; }

        // CSV queue-size logger (one sample per frame, including queue size = 0)
        try
        {
            string timestamp = DateTime.UtcNow.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
            _queueCsvLogPath = System.IO.Path.Combine(
                Application.persistentDataPath,
                $"received_queue_size_log_{timestamp}.csv");
            System.IO.File.WriteAllText(_queueCsvLogPath, "utc_iso,unix_ms,received_queue_size\n");
        }
        catch (Exception) { _queueCsvLogPath = null; }

        // CSV packet logger (one row per accepted packet)
        try
        {
            string timestamp = DateTime.UtcNow.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
            _packetCsvLogPath = System.IO.Path.Combine(Application.persistentDataPath, $"unity_packet_log_{timestamp}.csv");
            System.IO.File.WriteAllText(
                _packetCsvLogPath,
                "utc_iso,unix_ms,seq,cmd_seq_echo,t0_echo,t1_echo,t3_recv,tq_dequeue,t4_physics,t5_send,t6_unity\n");
        }
        catch (Exception) { _packetCsvLogPath = null; }

        // Start a short coroutine that waits for the Unity graphics device to be ready
        try
        {
            StartCoroutine(EnsureGraphicsReadyCoroutine());
        }
        catch (Exception) { /* ignore in non-play mode */ }
    }

    public void Connect()
    {
        string ip = (ipInput != null) ? ipInput.text : defaultIP;
        if (string.IsNullOrEmpty(ip)) ip = defaultIP;
        UpdateStatus("Connecting...", Color.yellow);

        try
        {
            _cmdClient = new TcpClient();
            _cmdClient.Connect(ip, sendPort);
            _cmdStream = _cmdClient.GetStream();

            _serverEndPoint = new IPEndPoint(IPAddress.Parse(ip), sendPort);
            
            if (_udpReceiver != null) _udpReceiver.Close();
            _udpReceiver = new UdpClient(listenPort);
            _udpReceiver.Client.ReceiveBufferSize = 8192;

            // Send probe packet from receive socket so server learns this port for state replies
            try
            {
                byte[] probe = Encoding.UTF8.GetBytes("{\"handshake_rx\":1}");
                _udpReceiver.Send(probe, probe.Length, _serverEndPoint);
                Debug.Log($"✅ Sent receive-side probe from port {listenPort}");
            }
            catch (Exception e)
            {
                Debug.LogWarning($"Failed to send receive-side probe: {e.Message}");
            }

            // Prepare video receiver only if video is enabled
            if (enableVideo)
            {
                try
                {
                    if (_videoReceiver != null) _videoReceiver.Close();
                    _videoReceiver = new UdpClient(videoListenPort);
                    _videoReceiver.Client.ReceiveBufferSize = 256 * 1024;
                    Debug.Log($"✅ Video UDP ready on {videoListenPort}");
                }
                catch (Exception ve)
                {
                    Debug.LogWarning($"Video receiver failed to initialize: {ve.Message}");
                    _videoReceiver = null;
                }
            }

            _lastReceivedSeq = -1;
            _isRunning = true;

            // Start video thread now that _isRunning is true (only if enabled)
            if (enableVideo && _videoReceiver != null)
            {
                try
                {
                    _videoThread = new Thread(VideoReceiveLoop);
                    _videoThread.IsBackground = true;
                    _videoThread.Start();
                    Debug.Log($"✅ Video UDP listening on {videoListenPort}");
                }
                catch (Exception ve)
                {
                    Debug.LogWarning($"Video receiver failed to start: {ve.Message}");
                    _videoReceiver = null;
                }
            }

            _receiveThread = new Thread(ReceiveLoop);
            _receiveThread.IsBackground = true;
            _receiveThread.Start();

            Debug.Log($"✅ Connected! TCP cmd:{sendPort} UDP state:{listenPort}");
            UpdateStatus("Connected!", Color.green);
            OnConnected?.Invoke();
            
            StartCoroutine(SendHandshakeLoop());
            // Panel stays visible — HidePanel removed
        }
        catch (Exception e)
        {
            Debug.LogError($"Connection Init Error: {e.Message}");
            UpdateStatus("Connection Failed", Color.red);
        }
    }

    // --- RECEIVE LOOP (The Engine) ---
    private void ReceiveLoop()
    {
        IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);
        while (_isRunning)
        {
            try
            {
                // Receive raw bytes — stamp T6 immediately, before any processing.
                byte[] data = _udpReceiver.Receive(ref remoteEndPoint);
                long t6 = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(); // T6: Unity UTC ms

                if (data == null || data.Length == 0) continue;
                string json = Encoding.UTF8.GetString(data);
                lock (_queueLock)
                {
                    // Store T6 alongside the JSON so Update() can apply it atomically.
                    _receivedQueue.Enqueue(json);
                    _pendingT6 = t6;
                }
            }
            catch (SocketException)
            {
                // Socket closed or interrupted; break if shutting down
                if (!_isRunning) break;
            }
            catch (ObjectDisposedException)
            {
                // Receiver disposed while blocking on Receive; exit loop
                break;
            }
            catch (Exception ex)
            {
                // Capture background exceptions for main-thread display and persistent log
                string msg = $"ReceiveLoop exception: {ex.GetType().Name}: {ex.Message}";
                lock (_debugLock) { _debugQueue.Enqueue(msg); }
                TryWritePersistentLog(msg);
                // Keep loop alive on unexpected errors
            }
        }
    }

    // Video receive loop: read chunked JPEG packets, reassemble by frame id and enqueue completed JPEGs
    private void VideoReceiveLoop()
    {
        IPEndPoint remote = new IPEndPoint(IPAddress.Any, 0);
        const int HEADER_SIZE = 12; // I(4) H(2) H(2) I(4)
        while (_isRunning)
        {
            try
            {
                byte[] data = _videoReceiver.Receive(ref remote);
                if (data == null || data.Length <= HEADER_SIZE) continue;

                // parse little-endian header
                uint frameId = BitConverter.ToUInt32(data, 0);
                ushort chunkIdx = BitConverter.ToUInt16(data, 4);
                ushort totalChunks = BitConverter.ToUInt16(data, 6);
                uint dataLen = BitConverter.ToUInt32(data, 8);

                int payloadLen = data.Length - HEADER_SIZE;
                if (payloadLen <= 0) continue;

                // copy payload
                byte[] chunkData = new byte[payloadLen];
                Buffer.BlockCopy(data, HEADER_SIZE, chunkData, 0, payloadLen);

                lock (_videoFramesLock)
                {
                    VideoFrameBuffer buf;
                    if (!_videoFrames.TryGetValue(frameId, out buf))
                    {
                        buf = new VideoFrameBuffer() { totalChunks = totalChunks, firstSeenTs = (DateTime.UtcNow - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).TotalSeconds };
                        _videoFrames[frameId] = buf;
                    }
                    // store chunk
                    if (!buf.chunks.ContainsKey(chunkIdx))
                    {
                        buf.chunks[chunkIdx] = chunkData;
                    }

                    // check completion
                    if (buf.chunks.Count == buf.totalChunks)
                    {
                        // assemble
                        int estimatedSize = 0;
                        for (int i = 0; i < buf.totalChunks; i++)
                        {
                            if (buf.chunks.TryGetValue(i, out var c)) estimatedSize += c.Length;
                        }
                        var ms = new System.IO.MemoryStream(estimatedSize);
                        for (int i = 0; i < buf.totalChunks; i++)
                        {
                            if (buf.chunks.TryGetValue(i, out var c)) ms.Write(c, 0, c.Length);
                        }
                        byte[] jpeg = ms.ToArray();

                        // enqueue decoded jpeg for main thread
                        lock (_videoLock)
                        {
                            if (_videoQueue.Count >= _maxVideoQueue)
                            {
                                // drop oldest
                                _videoQueue.Dequeue();
                            }
                            _videoQueue.Enqueue(jpeg);
                        }

                        // remove buffer
                        _videoFrames.Remove(frameId);
                    }
                }
                // cleanup stale buffers
                lock (_videoFramesLock)
                {
                    var keysToRemove = new List<uint>();
                    double now = (DateTime.UtcNow - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).TotalSeconds;
                    foreach (var kv in _videoFrames)
                    {
                        if (now - kv.Value.firstSeenTs > 2.0) keysToRemove.Add(kv.Key);
                    }
                    foreach (var k in keysToRemove) _videoFrames.Remove(k);
                }
            }
            catch (SocketException)
            {
                if (!_isRunning) break;
            }
            catch (ObjectDisposedException)
            {
                break;
            }
            catch (Exception ex)
            {
                string msg = $"VideoReceiveLoop exception: {ex.GetType().Name}: {ex.Message}";
                lock (_debugLock) { _debugQueue.Enqueue(msg); }
                TryWritePersistentLog(msg);
            }
        }
    }

    // Process queued packets on the Unity main thread (safe to use Unity APIs here)
    void Update()
    {
        // Drain the JSON queue into a local list to minimize lock time
        List<string> toProcess = null;
        long snappedT6 = 0;
        int queueSizeSnapshot = 0;
        lock (_queueLock)
        {
            queueSizeSnapshot = _receivedQueue.Count;
            if (_receivedQueue.Count > 0)
            {
                toProcess = new List<string>(_receivedQueue);
                _receivedQueue.Clear();
                snappedT6 = _pendingT6; // T6 paired to the batch
            }
        }

        // Log queue size every frame, including zero.
        TryAppendReceivedQueueCsv(queueSizeSnapshot);

        // NOTE: do NOT return early here — video processing must also run every frame
        if (toProcess != null)
        foreach (var json in toProcess)
        {
            try
            {
                IsaacState packet = JsonUtility.FromJson<IsaacState>(json);
                if (packet == null || packet.state == null || packet.state.Length != 6) continue;

                if (packet.seq > _lastReceivedSeq)
                {
                    _lastReceivedSeq = packet.seq;

                    // Stamp T6 into the packet object so it travels with all other fields
                    // as a single atomic unit when published to LatestState.
                    packet.t6_unity = snappedT6;

                    // Use the echoed cmd_seq to pair the response with the exact command that
                    // triggered it. Only update T1_ForLastResponse when Isaac echoes a cmd_seq
                    // we haven't consumed yet — this eliminates the stale T1 approximation.
                    if (packet.cmd_seq_echo > _lastConsumedCmdSeq && packet.t1_echo > 0)
                    {
                        _lastConsumedCmdSeq   = packet.cmd_seq_echo;
                        T1_ForLastResponse    = packet.t1_echo;   // exact matched T1 from Isaac echo
                        T0_InputCapture       = packet.t0_echo;   // exact matched T0 from Isaac echo
                    }
                    else
                    {
                        // Fallback: Isaac hasn't echoed a new cmd_seq yet (e.g. first few frames).
                        // Use static field snapshot as before — better than nothing.
                        T1_ForLastResponse    = T1_PacketSent;
                    }

                    T6_UnityReceived      = snappedT6;          // Unity UTC ms, stamped in ReceiveLoop
                    T3_IsaacReceived      = packet.t3_recv;     // Isaac UTC ms
                    TQ_IsaacDequeue       = packet.tq_dequeue;  // Isaac UTC ms
                    T4_PhysicsApplied     = packet.t4_physics;  // Isaac UTC ms
                    T5_IsaacSend          = packet.t5_send;     // Isaac UTC ms
                    LastReceivedSeqPublic = packet.seq;
                    LastReceivedCmdSeq    = packet.cmd_seq_echo;

                    // ── RFC 3550 jitter ──
                    double nowSec = GetUnixTimeSeconds();
                    long sendTs = packet.t5_send > 0 ? packet.t5_send : 0;
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

                    // Publish the complete, fully-populated state as a single atomic swap.
                    // LatencyLogger reads this under StateLock so all fields are always
                    // from the same packet — no torn reads, no mismatched pairs.
                    lock (StateLock)
                    {
                        LatestState = packet;
                    }

                    // Log one row per accepted packet.
                    TryAppendPacketCsv(packet);

                    for (int i = 0; i < 6; i++)
                    {
                        JointAnglesRad[i] = (packet.state[i] * jointSigns[i]) + (visualOffsets[i] * Mathf.Deg2Rad);
                    }

                    if (!_expectedInitialized)
                    {
                        lock (_expectedLock)
                        {
                            if (!_expectedInitialized)
                            {
                                Array.Copy(JointAnglesRad, _expectedAnglesRad, 6);
                                _expectedInitialized = true;
                            }
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"IsaacUR3Client: failed to process packet: {ex.Message}");
            }
        }

        // --- Video processing (main thread) ---
        byte[] frame = null;
        lock (_videoLock)
        {
            if (_videoQueue.Count > 0)
            {
                // drain and use the newest frame to reduce latency
                var list = new List<byte[]>(_videoQueue);
                _videoQueue.Clear();
                frame = list[list.Count - 1];
            }
        }

        if (frame != null)
        {
            try
            {
                // If graphics device isn't ready yet, defer the frame briefly
                if (!_graphicsReady)
                {
                    string msg = "Graphics device not ready - deferring video frame";
                    lock (_debugLock) { _debugQueue.Enqueue(msg); }
                    TryWritePersistentLog(msg);
                    // put the frame back (as newest) for a later attempt
                    lock (_videoLock)
                    {
                        _videoQueue.Enqueue(frame);
                        // cap queue to avoid unbounded growth
                        while (_videoQueue.Count > _maxVideoQueue) _videoQueue.Dequeue();
                    }
                }
                else
                {
                    if (_videoTexture == null)
                    {
                        _videoTexture = new Texture2D(2, 2, TextureFormat.RGBA32, false);
                        _videoTexture.wrapMode = TextureWrapMode.Clamp;
                    }
                    // LoadImage will resize the texture as needed
                    bool ok = ImageConversion.LoadImage(_videoTexture, frame);
                if (ok && videoDisplay != null)
                {
                    videoDisplay.texture = _videoTexture;
                }
                }
            }
            catch (Exception ex)
            {
                string msg = $"Video decode error: {ex.GetType().Name}: {ex.Message}";
                lock (_debugLock) { _debugQueue.Enqueue(msg); }
                TryWritePersistentLog(msg);
            }
        }
    }

    // Coroutine: wait until Unity's graphics device appears initialized
    System.Collections.IEnumerator EnsureGraphicsReadyCoroutine()
    {
        // Wait up to a few frames for the device to become available
        int attempts = 0;
        while (attempts < 60)
        {
            try
            {
                // Use a string-based check for compatibility across Unity versions
                // (some Unity installs or build targets may not expose the GraphicsDeviceType symbol)
                if (!string.Equals(SystemInfo.graphicsDeviceType.ToString(), "Null", StringComparison.OrdinalIgnoreCase))
                {
                    _graphicsReady = true;
                    yield break;
                }
            }
            catch (Exception)
            {
                // If reflection/string access fails for any reason, assume graphics is ready to avoid blocking
                _graphicsReady = true;
                yield break;
            }
            attempts++;
            yield return new WaitForEndOfFrame();
        }
        // Fallback: assume ready after waiting
        _graphicsReady = true;
    }

    void LateUpdate()
    {
        // Flush debug messages to on-screen text and log file
        if (_debugQueue.Count == 0) return;

        List<string> items = null;
        lock (_debugLock)
        {
            if (_debugQueue.Count > 0) { items = new List<string>(_debugQueue); _debugQueue.Clear(); }
        }

        if (items == null) return;

        string combined = string.Join("\n", items);
        if (debugText != null)
        {
            debugText.text = combined;
        }
        TryWritePersistentLog(combined);
    }

    // --- SENDING (Joystick Commands) ---
    // T0 = input-capture time (UTC ms), passed from the jog button on pointer-down.
    // T1 = send time stamped here just before sock.Send(), using the same UTC ms clock.
    public void SendDelta(int index, float amount, long t0 = 0)
    {
        NetworkStream stream = _cmdStream;
        if (stream != null && stream.CanWrite)
        {
            // T1: stamped just before the packet hits the wire — UTC ms, same clock as T6
            long t1 = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

            // Increment per-command sequence counter so Isaac can echo it back for exact pairing
            int cmdSeq = ++_cmdSeq;

            lock (_expectedLock)
            {
                if (!_expectedInitialized)
                {
                    Array.Copy(JointAnglesRad, _expectedAnglesRad, 6);
                    _expectedInitialized = true;
                }
                _expectedAnglesRad[index] += (amount * jointSigns[index]);
                float[] snapshot = new float[6];
                Array.Copy(_expectedAnglesRad, snapshot, 6);
                CmdToExpected[cmdSeq] = snapshot;
            }

            string json = string.Format(CultureInfo.InvariantCulture,
                "{{\"delta\": [{0}, {1}], \"t0\": {2}, \"t1\": {3}, \"cmd_seq\": {4}}}",
                index, amount, t0, t1, cmdSeq);
            byte[] bytes = Encoding.UTF8.GetBytes(json + "\n");
            try
            {
                lock (_cmdSendLock) { stream.Write(bytes, 0, bytes.Length); }
            }
            catch { }
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

    private System.Collections.IEnumerator ContinuousSendLoop()
    {
    
        while (_continuousSendActive)
        {
            long t0 = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            SendDelta(_continuousIndex, _continuousAmount, t0);
            yield return null;
        }
        _continuousSendRoutine = null;
    }
    
    System.Collections.IEnumerator SendHandshakeLoop()
    {
        while (_isRunning)
        {
            if (JointAnglesRad[0] == 0 && _lastReceivedSeq == -1) 
            {
                 string json = "{\"handshake\": 1}";
                 NetworkStream stream = _cmdStream;
                 if (stream != null && stream.CanWrite)
                 {
                     byte[] bytes = Encoding.UTF8.GetBytes(json + "\n");
                     try
                     {
                         lock (_cmdSendLock) { stream.Write(bytes, 0, bytes.Length); }
                     }
                     catch { }
                 }
            }
            yield return new WaitForSeconds(1.0f);
        }
    }

    private double GetUnixTimeSeconds()
    {
        return (DateTime.UtcNow - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).TotalSeconds;
    }

    void UpdateStatus(string message, Color color)
    {
        if (statusText != null) { statusText.text = message; statusText.color = color; }
    }
    void HidePanel() { if (connectionPanel != null) connectionPanel.SetActive(false); }

    public void DisconnectFromServer()
    {
        _isRunning = false;
        StopContinuousDelta();
        try { if (_udpReceiver != null) _udpReceiver.Close(); } catch (Exception) { }
        try { if (_videoReceiver != null) _videoReceiver.Close(); } catch (Exception) { }
        try { if (_cmdStream != null) _cmdStream.Close(); } catch (Exception) { }
        try { if (_cmdClient != null) _cmdClient.Close(); } catch (Exception) { }
        CmdToExpected.Clear();
        _expectedInitialized = false;
        UpdateStatus("Disconnected", Color.white);
        OnDisconnected?.Invoke();
    }

    void OnApplicationQuit()
    {
        _isRunning = false;
        StopContinuousDelta();
        try
        {
            if (_udpReceiver != null) _udpReceiver.Close();
        }
        catch (Exception) { }

        try
        {
            if (_videoReceiver != null) _videoReceiver.Close();
        }
        catch (Exception) { }

        try
        {
            if (_cmdStream != null) _cmdStream.Close();
        }
        catch (Exception) { }
        try
        {
            if (_cmdClient != null) _cmdClient.Close();
        }
        catch (Exception) { }

        CmdToExpected.Clear();
        _expectedInitialized = false;

        // Wait briefly for the receive thread to finish instead of aborting it
        try
        {
            if (_receiveThread != null && _receiveThread.IsAlive)
            {
                _receiveThread.Join(500);
            }
        }
        catch (Exception) { }
        try
        {
            if (_videoThread != null && _videoThread.IsAlive)
            {
                _videoThread.Join(500);
            }
        }
        catch (Exception) { }
    }

    private void TryWritePersistentLog(string message)
    {
        if (string.IsNullOrEmpty(_persistentLogPath)) return;
        try
        {
            System.IO.File.AppendAllText(_persistentLogPath, $"{DateTime.UtcNow:O} {message}\n");
        }
        catch (Exception) { }
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
            System.IO.File.AppendAllText(_queueCsvLogPath, row);
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
            System.IO.File.AppendAllText(_packetCsvLogPath, row);
        }
        catch (Exception) { }
    }
}
