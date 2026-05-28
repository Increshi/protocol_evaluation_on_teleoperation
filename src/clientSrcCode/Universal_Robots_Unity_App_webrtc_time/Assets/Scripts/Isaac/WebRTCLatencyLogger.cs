/**
 * WebRTCLatencyLogger.cs
 *
 * Mirrors LatencyLogger.cs but reads timing data from IsaacUR3WebRTCClient.
 * Keeps the same CSV schema so logs are comparable with the UDP pipeline.
 */

using System;
using System.IO;
using UnityEngine;

public class WebRTCLatencyLogger : MonoBehaviour
{
    [Header("Logging")]
    [Tooltip("Enable / disable CSV writing at runtime.")]
    public bool enableLogging = true;

    private StreamWriter _writer;
    private string _logPath;
    private int _lastLoggedCmdSeq = -1;

    void Start()
    {
        string timestamp = DateTime.UtcNow.ToString("yyyyMMdd_HHmmss");
        _logPath = Path.Combine(Application.persistentDataPath, $"latency_log_webrtc_{timestamp}.csv");
        Debug.Log($"[WebRTCLatencyLogger] Writing to: {_logPath}");

        try
        {
            _writer = new StreamWriter(
                new FileStream(_logPath, FileMode.Create, FileAccess.Write, FileShare.Read));
            _writer.AutoFlush = true;

            _writer.WriteLine(
                    "seq," +
                    "cmd_seq_echo," +
                    "T0_ms," +
                    "T1_ms," +
                    "T3_ms," +
                    "TQ_ms," +
                    "T4_ms," +
                    "T6_ms," +
                    "T7_ms," +
                    "RTT_ms," +
                    "Processing_ms," +
                    "QueueDelay_ms," +
                    "Network_ms," +
                    "RenderDelay_ms," +
                    "MTP_ms," +
                    "Jitter_ms");
        }
        catch (Exception e)
        {
            Debug.LogError($"[WebRTCLatencyLogger] Could not open log file: {e.Message}");
            enableLogging = false;
        }
    }

    void LateUpdate()
    {
        if (!enableLogging || _writer == null) return;
        
        // Grab the latest state safely
        IsaacUR3WebRTCClient.IsaacState snap;
        lock (IsaacUR3WebRTCClient.StateLock)
        {
            if (IsaacUR3WebRTCClient.LatestState == null) return;
            snap = IsaacUR3WebRTCClient.LatestState;
        }

        // Only log completely fresh incoming packets to avoid compounding render delay
        if (snap.cmd_seq_echo <= 0) return;
        if (snap.cmd_seq_echo == _lastLoggedCmdSeq) return;
        _lastLoggedCmdSeq = snap.cmd_seq_echo;

        // Ensure we only log when a command is actually echoing (for proper RTT)
        if (snap.t1_echo <= 0 || snap.t6_unity <= 0 || snap.cmd_seq_echo <= 0) return;

        long t7 = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

        long rtt = snap.t6_unity - snap.t1_echo;
        if (rtt < 0) rtt = 0;

        long processingMs = (snap.t4_physics > snap.tq_dequeue)
            ? (snap.t4_physics - snap.tq_dequeue) : 0L;

        long queueDelay = (snap.tq_dequeue > snap.t3_recv)
            ? (snap.tq_dequeue - snap.t3_recv) : 0L;

        long network = rtt - processingMs - queueDelay;
        if (network < 0) network = 0;

        // Render Delay = t7 (LateUpdate) - t6 (Time Unity picked the packet from the network queue)
        long renderDelay = t7 - snap.t6_unity;
        if (renderDelay < 0) renderDelay = 0;

        long mtp = t7 - snap.t1_echo;
        if (mtp < 0) mtp = 0;

        try
        {
            _writer.WriteLine(
                $"{snap.seq}," +
                $"{snap.cmd_seq_echo}," +
                $"{snap.t0_echo}," +
                $"{snap.t1_echo}," +
                $"{snap.t3_recv}," +
                $"{snap.tq_dequeue}," +
                $"{snap.t4_physics}," +
                $"{snap.t6_unity}," +
                $"{t7}," +
                $"{rtt}," +
                $"{processingMs}," +
                $"{queueDelay}," +
                $"{network}," +
                $"{renderDelay}," +
                $"{mtp}," +
                $"{IsaacUR3WebRTCClient.CurrentJitterMs:F4}");
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[WebRTCLatencyLogger] Write error: {e.Message}");
        }
    }

    void OnDestroy()
    {
        try { _writer?.Close(); } catch { }
    }

    void OnApplicationQuit()
    {
        try { _writer?.Close(); } catch { }
    }
}
