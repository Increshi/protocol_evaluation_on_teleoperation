using UnityEngine;
using System;
using System.Text;
using System.Net.Sockets;
using System.Threading;
using System.Collections.Concurrent;
using System.Globalization;
using System.IO;
using TMPro;

// =============================================================================
// IsaacUR3Client — TCP transport, spec-compliant timestamp pipeline
//
// Timestamp roles (all public statics consumed by LatencyLogger):
//   T0  DateTimeOffset.UtcNow ms  — input captured (set by IsaacJogButton, echoed back via t0_echo)
//   T1  DateTimeOffset.UtcNow ms  — packet sent (set here in SendDelta, echoed back via t1_echo)
//   T3  UTC ms (Isaac)           — Isaac recvfrom() [ms, echoed back]
//   T4  UTC ms (Isaac)           — Isaac physics applied [ms, echoed back]
//   T5  UTC ms (Isaac)           — Isaac sendto() [ms, echoed back]
//   TQ  UTC ms (Isaac)           — Isaac queue dequeue [ms, echoed back]
//   T6  DateTimeOffset.UtcNow ms  — Unity TCP read (set here in ReceiveLoop)
//   T7  DateTimeOffset.UtcNow ms  — frame rendered (set by LatencyLogger)
// =============================================================================
public class IsaacUR3Client : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // Public static timestamps — read every frame by LatencyLogger
    // -------------------------------------------------------------------------
    // Unity-side: UTC milliseconds (DateTimeOffset.UtcNow.ToUnixTimeMilliseconds())
    public static long T0_InputCapture   = 0; // set by IsaacJogButton.OnPointerDown()
    public static long T1_PacketSent     = 0; // set here in SendDelta(), just before stream.Write()
    public static long T6_UnityReceived  = 0; // set here in ReceiveLoop(), just after stream.Read()
    // Isaac-side: UTC milliseconds (chrony/system clock)
    public static long T3_IsaacReceived  = 0; // echoed from Isaac recvfrom() timestamp [ms]
    public static long TQ_IsaacDequeue   = 0; // echoed from Isaac queue dequeue timestamp [ms]
    public static long T4_PhysicsApplied = 0; // echoed from Isaac physics applied [ms]
    public static long T5_IsaacSend      = 0; // echoed from Isaac sendto() timestamp [ms]

    // T1 matched to the specific command whose response just arrived.
    // Isaac echoes back t1_echo + cmd_seq_echo so we always have an exact pair,
    // even if multiple commands were in-flight (TCP buffering / Nagle).
    public static long T1_ForLastResponse = 0;

    // Sequence tracking — used by LatencyLogger's cmd_seq pairing guard
    public static int LastReceivedSeqPublic = -1;
    public static int LastReceivedCmdSeq    = -1;
    public static float CurrentJitterMs     = 0f; // RFC 3550-style inter-arrival jitter

    // -------------------------------------------------------------------------
    // Serialisable JSON classes
    // -------------------------------------------------------------------------
    // Command sent Unity → Isaac
    [Serializable]
    public class IsaacDeltaCommand
    {
        public float[] delta;
        public long    t1;       // T1: UTC ms, stamped just before stream.Write()
        public int     cmd_seq;  // monotone counter so Isaac can echo it back
    }

    // State received Isaac → Unity
    [Serializable]
    public class IsaacState
    {
        public float[] state;
        public int     seq;           // Isaac frame counter
        public int     cmd_seq_echo;  // echoed Unity cmd_seq for exact T1 pairing
        public long    t0_echo;       // echoed Unity T0 UTC ms — for MTP
        public long    t1_echo;       // echoed Unity T1 UTC ms
        public long    t3_recv;       // T3: Isaac UTC ms
        public long    tq_dequeue;    // TQ: Isaac UTC ms
        public long    t4_physics;    // T4: Isaac UTC ms
        public long    t5_send;       // T5: Isaac UTC ms
    }

    // -------------------------------------------------------------------------
    // Inspector fields
    // -------------------------------------------------------------------------
    [Header("UI Assignments")]
    public TMP_InputField ipInput;
    public TMP_Text       statusText;
    public GameObject     connectionPanel;

    [Header("Network Configuration")]
    public string defaultIP   = "10.9.71.137";
    public int    commandPort = 11020;
    public int    statePort   = 11021;

    [Header("Robot Settings")]
    public float[] visualOffsets = new float[6];
    public float[] jointSigns    = new float[] { 1, 1, 1, 1, 1, 1 };

    // -------------------------------------------------------------------------
    // Shared joint data — read by ur3_link1-6 scripts
    // -------------------------------------------------------------------------
    public static float[] JointAnglesRad = new float[6];
    public static ConcurrentDictionary<int, float[]> CmdToExpected = new ConcurrentDictionary<int, float[]>();

    private static float[] _expectedAnglesRad = new float[6];
    private static bool _expectedInitialized = false;
    private static readonly object _expectedLock = new object();

    // -------------------------------------------------------------------------
    // Connection-state events (subscribed by main_ui_control)
    // -------------------------------------------------------------------------
    public static event Action OnConnected;
    public static event Action OnDisconnected;

    // -------------------------------------------------------------------------
    // Internal TCP state
    // -------------------------------------------------------------------------
    private TcpClient     _cmdClient;
    private TcpClient     _stateClient;
    private NetworkStream _cmdStream;
    private NetworkStream _stateStream;

    private Thread _receiveThread;
    private Thread _connectThread;
    private bool   _isConnected = false;
    private bool   _isAlive     = false;

    // Per-command sequence counter — incremented on every SendDelta() call
    private int _cmdSeq          = 0;
    // Last cmd_seq whose echo has been applied to T1_ForLastResponse
    private int _lastConsumedCmdSeq = -1;

    // UI control messages from background thread → main thread
    private ConcurrentQueue<string> _messageQueue = new ConcurrentQueue<string>();

    // ---- Per-packet carry: receive-thread → Update ----
    // Struct pairs the parsed IsaacState with the T6 stamp taken in ReceiveLoop.
    private struct PendingPacket
    {
        public IsaacState data;
        public long       t6; // Unity UTC ms, stamped immediately after stream.Read()
    }
    private ConcurrentQueue<PendingPacket> _packetQueue = new ConcurrentQueue<PendingPacket>();
    private int _packetQueueCount = 0;

    private string _queueCsvLogPath = null;
    private string _packetCsvLogPath = null;

    // TCP reassembly buffer (TCP may split or coalesce packets)
    private string _incompleteMessage = "";

    // ── Application-level jitter (RFC 3550) for state packets ──────────────
    // D(i) = recv_interval - send_interval
    // J(i) = J(i-1) + (|D(i)| - J(i-1)) / 16
    private long _prevPktSendTs = -1;    // sender timestamp (ms) of previous packet
    private double _prevPktRecvTime = 0; // local receive time (sec) of previous packet
    private float _stateJitterMs = 0f;   // smoothed jitter estimate (ms)

    // =========================================================================
    void Start()
    {

        _isAlive = true;
        if (ipInput != null) ipInput.text = defaultIP;
        UpdateStatus("Ready to Connect", Color.white);

        // CSV queue-size logger (one sample per frame, including queue size = 0)
        try
        {
            string timestamp = DateTime.UtcNow.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
            _queueCsvLogPath = Path.Combine(
                Application.persistentDataPath,
                $"received_queue_size_log_{timestamp}.csv");
            File.WriteAllText(_queueCsvLogPath, "utc_iso,unix_ms,received_queue_size\n");
        }
        catch (Exception)
        {
            _queueCsvLogPath = null;
        }

        // CSV packet logger (one row per accepted packet)
        try
        {
            string timestamp = DateTime.UtcNow.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
            _packetCsvLogPath = Path.Combine(
                Application.persistentDataPath,
                $"unity_packet_log_{timestamp}.csv");
            File.WriteAllText(
                _packetCsvLogPath,
                "utc_iso,unix_ms,seq,cmd_seq_echo,t0_echo,t1_echo,t3_recv,tq_dequeue,t4_physics,t5_send,t6_unity\n");
        }
        catch (Exception)
        {
            _packetCsvLogPath = null;
        }
    }

    // =========================================================================
    // Connection
    // =========================================================================
    public void Connect()
    {
        string targetIP = defaultIP;
        if (ipInput != null && !string.IsNullOrEmpty(ipInput.text))
            targetIP = ipInput.text.Trim();

        UpdateStatus($"Connecting to {targetIP}...", Color.yellow);

        _connectThread = new Thread(() => ConnectLogic(targetIP));
        _connectThread.IsBackground = true;
        _connectThread.Start();
    }

    private void ConnectLogic(string ip)
    {
        try
        {
            _cmdClient = new TcpClient();
            _cmdClient.Connect(ip, commandPort);
            _cmdStream = _cmdClient.GetStream();
            Debug.Log($"[IsaacUR3Client] Command TCP connected: {ip}:{commandPort}");

            _stateClient = new TcpClient();
            _stateClient.Connect(ip, statePort);
            _stateStream = _stateClient.GetStream();
            Debug.Log($"[IsaacUR3Client] State TCP connected: {ip}:{statePort}");

            _isConnected = true;

            _receiveThread = new Thread(ReceiveLoop);
            _receiveThread.IsBackground = true;
            _receiveThread.Start();

            _messageQueue.Enqueue("UI_CONNECTED");
        }
        catch (Exception e)
        {
            Debug.LogError($"[IsaacUR3Client] Connect failed: {e.Message}");
            _messageQueue.Enqueue("UI_FAILED: " + e.Message);
            DisconnectFromServer();
        }
    }

    public void DisconnectFromServer()
    {
        _isConnected = false;
        try { _cmdStream?.Close();   } catch { }
        try { _cmdClient?.Close();   } catch { }
        try { _stateStream?.Close(); } catch { }
        try { _stateClient?.Close(); } catch { }

        CmdToExpected.Clear();
        _expectedInitialized = false;

        if (_receiveThread != null && _receiveThread.IsAlive)
            _receiveThread.Abort();

        if (_isAlive)
            _messageQueue.Enqueue("UI_DISCONNECTED");
    }

    // =========================================================================
    // SendDelta — stamps T1, embeds t0 + t1 + cmd_seq, sends over command TCP stream
    // =========================================================================
    // t0 is passed in from IsaacJogButton (the input-capture time, UTC ms).
    // T0 travels to Isaac so Isaac can echo it back as t0_echo, which is used
    // to update T0_InputCapture on the matched response (same flow as t1/t1_echo).
    public void SendDelta(int index, float amount, long t0 = 0)
    {
        if (!_isConnected || _cmdStream == null || !_cmdStream.CanWrite) return;
        try
        {
            // T1: stamp just before the bytes hit the stream — UTC ms, same clock as T6
            long t1 = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            T1_PacketSent = t1;

            // Increment per-command sequence counter
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

            // Build command JSON manually to avoid JsonUtility limitations with long
            string json = string.Format(CultureInfo.InvariantCulture,
                "{{\"delta\":[{0},{1}],\"t0\":{2},\"t1\":{3},\"cmd_seq\":{4}}}\n",
                index, amount, t0, t1, cmdSeq);

            byte[] bytes = Encoding.UTF8.GetBytes(json);
            _cmdStream.Write(bytes, 0, bytes.Length);
        }
        catch (Exception e) { Debug.LogError($"[IsaacUR3Client] Send Error: {e.Message}"); }
    }

    // =========================================================================
    // ReceiveLoop — background thread
    // T6 is stamped once per logical message (per complete JSON line), not per
    // raw Read() call, because TCP may return multiple lines in one Read().
    // =========================================================================
    private void ReceiveLoop()
    {
        byte[] buffer = new byte[8192];
        while (_isConnected && _stateStream != null)
        {
            try
            {
                int n = _stateStream.Read(buffer, 0, buffer.Length);
                if (n <= 0)
                {
                    Debug.LogWarning("[IsaacUR3Client] State TCP closed by server (EOF)." );
                    break; // server closed connection cleanly (graceful EOF)
                }

                // T6: UTC ms, stamped immediately after bytes arrive from the wire
                long t6 = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

                string chunk = Encoding.UTF8.GetString(buffer, 0, n);
                _incompleteMessage += chunk;

                // Extract all complete newline-terminated JSON lines
                while (_incompleteMessage.Contains("\n"))
                {
                    int nl   = _incompleteMessage.IndexOf("\n");
                    string line = _incompleteMessage.Substring(0, nl).Trim();
                    _incompleteMessage = _incompleteMessage.Substring(nl + 1);
                    if (string.IsNullOrEmpty(line)) continue;

                    // Each complete line gets the T6 of the Read() that delivered it.
                    try
                    {
                        IsaacState pkt = JsonUtility.FromJson<IsaacState>(line);
                        if (pkt != null && pkt.state != null && pkt.state.Length == 6)
                        {
                            _packetQueue.Enqueue(new PendingPacket { data = pkt, t6 = t6 });
                            Interlocked.Increment(ref _packetQueueCount);
                            continue;
                        }
                    }
                    catch { /* malformed JSON line — skip, keep connection alive */ }

                    // Non-state messages fall through to the UI queue
                    _messageQueue.Enqueue(line);
                }
            }
            catch (System.IO.IOException ioEx)
            {
                // IOException wraps SocketException for TCP reads.
                Debug.LogWarning($"[IsaacUR3Client] State TCP read IOException: {ioEx.Message}");
                // Only break on errors that mean the connection is truly gone.
                var inner = ioEx.InnerException as System.Net.Sockets.SocketException;
                if (inner != null)
                {
                    var code = inner.SocketErrorCode;
                    Debug.LogWarning($"[IsaacUR3Client] SocketError: {code}");
                    if (code == System.Net.Sockets.SocketError.ConnectionReset  ||
                        code == System.Net.Sockets.SocketError.ConnectionAborted ||
                        code == System.Net.Sockets.SocketError.Shutdown         ||
                        code == System.Net.Sockets.SocketError.NotConnected)
                    {
                        break; // real disconnection
                    }
                    // Transient errors (e.g. WouldBlock, Interrupted) — keep looping
                    continue;
                }
                break; // unknown IOException — treat as disconnection
            }
            catch (ObjectDisposedException) { break; } // socket was closed by Disconnect
            catch (Exception)              { break; } // any other unexpected error
        }

        if (_isAlive)
            _messageQueue.Enqueue("UI_DISCONNECTED");
    }

    // =========================================================================
    // Update — main thread: drain queues, apply joints, populate public statics
    // =========================================================================
    void Update()
    {
        // Log queue size at the start of the frame, before draining.
        int queueSizeSnapshot = Interlocked.Add(ref _packetQueueCount, 0);
        TryAppendReceivedQueueCsv(queueSizeSnapshot);

        // ---- UI control messages ----
        while (_messageQueue.TryDequeue(out string message))
        {
            if (message == "UI_CONNECTED")
            {
                UpdateStatus("Connected!", Color.green);
                OnConnected?.Invoke();
                continue;
            }
            if (message.StartsWith("UI_FAILED"))
            {
                UpdateStatus("Connection Failed", Color.red);
                OnDisconnected?.Invoke();
                continue;
            }
            if (message == "UI_DISCONNECTED")
            {
                UpdateStatus("Disconnected", Color.white);
                OnDisconnected?.Invoke();
                continue;
            }
        }

        // ---- State packets ----
        // Drain all queued packets; keep only the newest for this frame.
        PendingPacket latest  = default;
        bool          hasPacket = false;
        while (_packetQueue.TryDequeue(out PendingPacket p))
        {
            latest    = p;
            hasPacket = true;
            Interlocked.Decrement(ref _packetQueueCount);

            // Log every dequeued packet (not just the most recent).
            TryAppendPacketCsv(p.data, p.t6);
        }

        if (hasPacket)
        {
            IsaacState pkt = latest.data;

            // ---- cmd_seq pairing guard ----
            // Only update T1_ForLastResponse when Isaac echoes a cmd_seq we haven't
            // consumed yet. This ensures T1_ForLastResponse is the exact T1 that
            // corresponds to the command that triggered this response — no stale rows.
            if (pkt.cmd_seq_echo > _lastConsumedCmdSeq && pkt.t1_echo > 0)
            {
                _lastConsumedCmdSeq = pkt.cmd_seq_echo;
                T1_ForLastResponse  = pkt.t1_echo; // exact matched T1 echoed by Isaac
                // Replace local T0 with the echoed value so MTP uses the server-confirmed
                // input-capture time rather than the locally-stored one.
                if (pkt.t0_echo > 0) T0_InputCapture = pkt.t0_echo;
            }
            else
            {
                // Fallback for the first few frames before Isaac has echoed any cmd_seq
                T1_ForLastResponse = T1_PacketSent;
            }

            // ---- Populate shared timestamp statics ----
            T6_UnityReceived      = latest.t6;       // UTC ms, stamped in ReceiveLoop
            T3_IsaacReceived      = pkt.t3_recv;     // Isaac UTC ms
            TQ_IsaacDequeue       = pkt.tq_dequeue;  // Isaac UTC ms
            T4_PhysicsApplied     = pkt.t4_physics;  // Isaac UTC ms
            T5_IsaacSend          = pkt.t5_send;     // Isaac UTC ms
            LastReceivedSeqPublic = pkt.seq;
            LastReceivedCmdSeq    = pkt.cmd_seq_echo;

            // ── RFC 3550 jitter ──
            double nowSec = GetUnixTimeSeconds();
            long sendTs = pkt.t5_send > 0 ? pkt.t5_send : 0;
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

            // ---- Apply joint angles (read by ur3_link1-6) ----
            for (int i = 0; i < 6; i++)
                JointAnglesRad[i] = (pkt.state[i] * jointSigns[i])
                                    + (visualOffsets[i] * Mathf.Deg2Rad);

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

    // =========================================================================
    // Helpers
    // =========================================================================
    private double GetUnixTimeSeconds()
    {
        return (DateTime.UtcNow - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).TotalSeconds;
    }

    void UpdateStatus(string message, Color color)
    {
        if (statusText != null) { statusText.text = message; statusText.color = color; }
    }

    private void OnDestroy()
    {
        _isAlive = false;
        DisconnectFromServer();
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
        catch (Exception)
        {
        }
    }

    private void TryAppendPacketCsv(IsaacState packet, long t6Unity)
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
                t6Unity);
            File.AppendAllText(_packetCsvLogPath, row);
        }
        catch (Exception)
        {
        }
    }
}
