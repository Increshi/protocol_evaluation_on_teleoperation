/**
 * LatencyLogger.cs  —  TCP variant
 *
 * Measures and logs end-to-end latency for the Isaac ↔ Unity TCP pipeline.
 *
 * Timing model
 * ─────────────────────────────────────────────────────────────────────────
 *  T0  Input captured    — IsaacJogButton.OnPointerDown()      [Unity UTC ms]
 *  T1  Packet sent       — IsaacUR3Client.SendDelta()          [Unity UTC ms]
 *  T3  Isaac received    — Isaac Python recvfrom() timestamp   [Isaac UTC ms]
 *  TQ  Isaac dequeued    — Isaac queue dequeue timestamp       [Isaac UTC ms]
 *  T4  Physics applied   — Isaac Python after set_joint_pos    [Isaac UTC ms]
 *  T5  Isaac sent        — Isaac Python before sendto()        [Isaac UTC ms]
 *  T6  Unity received    — IsaacUR3Client.ReceiveLoop()        [Unity UTC ms]
 *  T7  Frame rendered    — LateUpdate() below                  [Unity UTC ms]
 *
 *  RTT          = T6 − T1_ForLastResponse   [ms]  exact matched pair via cmd_seq echo
 *  Processing   = (T4 − T3)                 [ms]  Isaac clock only — always valid
 *  Queue Delay  = (TQ − T3)                 [ms]  Isaac clock only — always valid
 *  Network      = RTT − Processing − Queue  [ms]  approximation, clamped ≥ 0
 *  Render Delay = T7 − T6                   [ms]  Unity clock only
 *  MTP          = T7 − T1_ForLastResponse   [ms]  Motion-to-Photon, Unity clock only
 *
 * Guard: a CSV row is written ONLY when cmd_seq_echo changes to a new value.
 *   cmd_seq is incremented by Unity on every SendDelta() call and echoed back
 *   by Isaac in the TCP response. When the echo changes, Isaac definitely processed
 *   a new command — no stale rows possible.
 *
 * Output: Application.persistentDataPath/latency_log_<timestamp>.csv  (new file each Play)
 *
 * Setup: attach this component to any persistent GameObject in the scene.
 * ─────────────────────────────────────────────────────────────────────────
 */

using System;
using System.IO;
using UnityEngine;

public class LatencyLogger : MonoBehaviour
{
    [Header("Logging")]
    [Tooltip("Enable / disable CSV writing at runtime.")]
    public bool enableLogging = true;

    // ── internal state ───────────────────────────────────────────────────────
    private StreamWriter _writer;
    private string       _logPath;
    private int          _lastLoggedCmdSeq = -1; // cmd_seq_echo at the last logged row

    // ── per-frame snapshot captured in Update() ──────────────────────────────
    // Snapshotting in Update() and writing in LateUpdate() ensures all values
    // in a single CSV row belong to the same packet, even if the receive thread
    // updates the statics mid-frame.
    private long _t0_snap;
    private long _t1rtt_snap;      // T1_ForLastResponse: exact echoed T1 from Isaac
    private long _t3_snap;
    private long _tq_snap;
    private long _t4_snap;
    private long _t5_snap;
    private long _t6_snap;
    private int  _seq_snap;
    private int  _cmdSeqEcho_snap; // cmd_seq_echo — the pairing key

    // ─────────────────────────────────────────────────────────────────────────

    void Start()
    {
        string timestamp = DateTime.UtcNow.ToString("yyyyMMdd_HHmmss");
        _logPath = Path.Combine(Application.persistentDataPath, $"latency_log_{timestamp}.csv");
        Debug.Log($"[LatencyLogger] Writing to: {_logPath}");

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
                "T3_ns," +
                "TQ_ms," +
                "T4_ns," +
                "T6_ms," +
                "T7_ms," +
                "RTT_ms," +
                "Processing_ns," +
                "QueueDelay_ms," +
                "Network_ms," +
                "RenderDelay_ms," +
                "MTP_ms," +
                "Jitter_ms"
            );
        }
        catch (Exception e)
        {
            Debug.LogError($"[LatencyLogger] Could not open log file: {e.Message}");
            enableLogging = false;
        }
    }

    // Snapshot all shared statics early in the frame so every value in this
    // row belongs to the same packet, even if the receive thread updates
    // them during the frame.
    void Update()
    {
        _t0_snap          = IsaacUR3Client.T0_InputCapture;
        _t1rtt_snap       = IsaacUR3Client.T1_ForLastResponse; // exact echoed T1 from Isaac
        _t3_snap          = IsaacUR3Client.T3_IsaacReceived;
        _tq_snap          = IsaacUR3Client.TQ_IsaacDequeue;
        _t4_snap          = IsaacUR3Client.T4_PhysicsApplied;
        _t5_snap          = IsaacUR3Client.T5_IsaacSend;
        _t6_snap          = IsaacUR3Client.T6_UnityReceived;
        _seq_snap         = IsaacUR3Client.LastReceivedSeqPublic;
        _cmdSeqEcho_snap  = IsaacUR3Client.LastReceivedCmdSeq;
    }

    // LateUpdate fires after all transforms are updated — T7 is as close as
    // possible to the frame actually appearing on screen.
    void LateUpdate()
    {
        if (!enableLogging || _writer == null) return;

        // Skip until we have real send and receive data
        if (_t1rtt_snap <= 0 || _t6_snap <= 0) return;

        // ── cmd_seq pairing guard ─────────────────────────────────────────────
        //
        // A row is ONLY written when cmd_seq_echo changes to a new value.
        // cmd_seq is incremented by Unity on every SendDelta() call. Isaac echoes
        // it back in the TCP response. When the echo changes:
        //   • Isaac definitely received and processed a new command this cycle
        //   • T1_ForLastResponse is the exact T1 for that command
        //   • No stale rows are possible — one row per unique command round-trip
        if (_cmdSeqEcho_snap <= 0)                     return; // Isaac hasn't echoed any cmd yet
        if (_cmdSeqEcho_snap == _lastLoggedCmdSeq)     return; // same cmd_seq, already logged

        _lastLoggedCmdSeq = _cmdSeqEcho_snap;

        // T7: frame rendered — UTC ms, Unity clock only
        long t7 = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

        // ── Metrics ───────────────────────────────────────────────────────────

        // RTT: T1_ForLastResponse and T6 are a matched send/receive pair on the
        // same Unity UTC ms clock — always valid and always positive.
        long rtt = _t6_snap - _t1rtt_snap;
        if (rtt < 0) rtt = 0;

        // Processing: Isaac physics window — T3 and T4 are on the same Isaac clock.
        long processingMs = (_t4_snap > _tq_snap) ? (_t4_snap - _tq_snap) : 0L;

        // Queue delay: Isaac dequeue wait (TQ - T3), UTC ms on same Isaac clock.
        long queueDelay = (_tq_snap > _t3_snap) ? (_tq_snap - _t3_snap) : 0L;

        // Network: approximation of total wire time, clamped ≥ 0
        long network = rtt - processingMs - queueDelay;
        if (network < 0) network = 0;

        // Render Delay: time from packet arrival to this frame being rendered
        long renderDelay = t7 - _t6_snap;
        if (renderDelay < 0) renderDelay = 0;

        // MTP: Motion-to-Photon — from packet-send to rendered frame
        long mtp = (_t1rtt_snap > 0) ? (t7 - _t1rtt_snap) : 0L;
        if (mtp < 0) mtp = 0;

        // ── Write CSV row ─────────────────────────────────────────────────────
        try
        {
            _writer.WriteLine(
                $"{_seq_snap}," +
                $"{_cmdSeqEcho_snap}," +
                $"{_t0_snap}," +
                $"{_t1rtt_snap}," +
                $"{_t3_snap}," +
                $"{_tq_snap}," +
                $"{_t4_snap}," +
                $"{_t6_snap}," +
                $"{t7}," +
                $"{rtt}," +
                $"{processingMs}," +
                $"{queueDelay}," +
                $"{network}," +
                $"{renderDelay}," +
                $"{mtp}," +
                $"{IsaacUR3Client.CurrentJitterMs:F4}"
            );
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[LatencyLogger] Write error: {e.Message}");
        }
    }

    void OnDestroy()
    {
        try { _writer?.Close(); } catch { /* ignore */ }
    }

    void OnApplicationQuit()
    {
        try { _writer?.Close(); } catch { /* ignore */ }
    }
}
