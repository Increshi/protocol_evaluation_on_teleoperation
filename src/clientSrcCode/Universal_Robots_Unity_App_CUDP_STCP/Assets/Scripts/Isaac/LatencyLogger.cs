/**
 * LatencyLogger.cs
 *
 * Measures and logs end-to-end latency for the Isaac ↔ Unity UDP pipeline.
 *
 * Timing model
 * ─────────────────────────────────────────────────────────────────────────
 *  T0  Input captured   – stamped in VRRobotInput on joystick rising edge [Unity UTC ms]
 *  T1  Packet sent      – stamped in IsaacUR3Client.SendDelta()        [Unity UTC ms]
 *  T3  Isaac received   – stamped in Isaac Python recvfrom()           [Isaac monotonic ns]
 *  T4  Physics applied  – stamped in Isaac Python after set_joint_pos  [Isaac monotonic ns]
 *  T5  Isaac sent       – stamped in Isaac Python before sendto()      [Isaac monotonic ns]
 *  T6  Unity received   – stamped in IsaacUR3Client.ReceiveLoop()      [Unity UTC ms]
 *  T7  Frame rendered   – stamped in LateUpdate() below                [Unity UTC ms]
 *
 *  RTT          = T6 − T1_echo                  [ms]  exact matched pair via cmd_seq echo
 *  Processing   = (T4 − T3) / 1,000,000         [ms]  Isaac clock only — always valid
 *  Network      = RTT − Processing              [ms]  approximation, clamped ≥ 0
 *  Render Delay = T7 − T6                       [ms]  Unity clock only
 *  MTP          = T7 − T1_echo                  [ms]  Motion-to-Photon, Unity clock only
 *
 * Thread-safety
 * ─────────────────────────────────────────────────────────────────────────
 *  All timing fields are read from IsaacUR3Client.LatestState, which is an
 *  object reference swapped atomically under IsaacUR3Client.StateLock in
 *  IsaacUR3Client.Update() (main thread). LatencyLogger.Update() acquires the
 *  same lock to copy the reference — after that the local snapshot is
 *  immutable (the background thread never mutates an already-published object).
 *  This means all fields (t0_echo, t1_echo, t3, t4, t5, t6, seq, cmd_seq_echo)
 *  always come from the SAME packet — no torn reads, no mismatched pairs.
 *
 * Guard
 * ─────────────────────────────────────────────────────────────────────────
 *  A CSV row is only written when cmd_seq_echo advances to a new value.
 *  cmd_seq is incremented by Unity on every SendDelta() call and echoed back
 *  by Isaac in the response. One row per unique completed command round-trip.
 *
 * Output: Application.persistentDataPath/latency_log_<timestamp>.csv  (new file each Play)
 * ─────────────────────────────────────────────────────────────────────────
 *
 * Setup: attach this component to any persistent GameObject in the scene.
 */

using System;
using System.IO;
using UnityEngine;

public class LatencyLogger : MonoBehaviour
{
    [Header("Logging")]
    [Tooltip("Enable / disable CSV writing at runtime.")]
    public bool enableLogging = true;

    // ── internal state ──────────────────────────────────────────────────────
    private StreamWriter _writer;
    private string       _logPath;
    private int          _lastLoggedCmdSeq  = -1; // cmd_seq_echo at last logged row

    // ── per-frame snapshot captured in Update() ─────────────────────────────
    // Taken from IsaacUR3Client.LatestState under StateLock — all fields are
    // guaranteed to be from the same packet.
    private IsaacUR3Client.IsaacState _snap = null;

    // ────────────────────────────────────────────────────────────────────────

    void Start()
    {
        // New timestamped file for every Play session — no appending across runs.
        // Format: latency_log_20260311_153042.csv
        string timestamp = DateTime.UtcNow.ToString("yyyyMMdd_HHmmss");
        _logPath = Path.Combine(Application.persistentDataPath, $"latency_log_{timestamp}.csv");
        Debug.Log($"[LatencyLogger] Writing to: {_logPath}");

        try
        {
            _writer = new StreamWriter(
                new FileStream(_logPath, FileMode.Create, FileAccess.Write, FileShare.Read));
            _writer.AutoFlush = true;

            // Always write header — file is always new
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
                    "Jitter_ms");
        }
        catch (Exception e)
        {
            Debug.LogError($"[LatencyLogger] Could not open log file: {e.Message}");
            enableLogging = false;
        }
    }

    // Snapshot the latest published state early in the frame.
    // Acquiring StateLock here is safe: IsaacUR3Client.Update() also runs on the
    // main thread, so there is no actual contention — the lock just guarantees
    // we copy the reference atomically and see a fully-written object.
    void Update()
    {
        lock (IsaacUR3Client.StateLock)
        {
            _snap = IsaacUR3Client.LatestState; // copy reference — object itself is immutable after publish
        }
    }

    // LateUpdate fires after all transforms are updated — T7 is as close
    // as possible to the frame actually appearing on screen.
    void LateUpdate()
    {
        if (!enableLogging || _writer == null) return;
        if (_snap == null) return;

        // Skip until we have real echoed send data
        if (_snap.t1_echo <= 0 || _snap.t6_unity <= 0) return;

        // ── cmd_seq pairing guard ────────────────────────────────────────────
        // A row is ONLY written when cmd_seq_echo advances to a new value.
        // cmd_seq is incremented by Unity on every SendDelta() call and echoed
        // back by Isaac. When the echo changes, Isaac definitely received a new
        // command this cycle — one row per unique completed command round-trip.
        if (_snap.cmd_seq_echo <= 0)                      return; // no echo yet
        if (_snap.cmd_seq_echo == _lastLoggedCmdSeq)      return; // already logged this cmd

        _lastLoggedCmdSeq = _snap.cmd_seq_echo;

        // T7: frame rendered — UTC ms, Unity clock only
        long t7 = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

        // ── Metrics ─────────────────────────────────────────────────────────

        // RTT: t1_echo and t6_unity are a matched send/receive pair on the same
        // Unity UTC ms clock — always valid and always positive.
        long rtt = _snap.t6_unity - _snap.t1_echo;
        if (rtt < 0) rtt = 0;

        // Processing: Isaac physics window — t3_recv and t4_physics are both
        // time.monotonic_ns() on the same Isaac machine. Stored in ns — do NOT
        // divide to ms here, sub-ms steps would truncate to zero.
        long processingMs = (_snap.t4_physics > _snap.tq_dequeue)
            ? (_snap.t4_physics - _snap.tq_dequeue) : 0L;// ms used only for Network approximation

        // Queue delay: Isaac dequeue wait (TQ - T3), UTC ms on same Isaac clock.
        long queueDelay = (_snap.tq_dequeue > _snap.t3_recv)
            ? (_snap.tq_dequeue - _snap.t3_recv) : 0L;

        // Network: approximation of total wire time, clamped >= 0
        long network = rtt - processingMs - queueDelay;
        if (network < 0) network = 0;

        // Render Delay: time from packet arrival to this frame being rendered
        long renderDelay = t7 - _snap.t6_unity;
        if (renderDelay < 0) renderDelay = 0;

        // MTP: Motion-to-Photon — from packet-send to frame rendered
        long mtp = t7 - _snap.t1_echo;
        if (mtp < 0) mtp = 0;

        // ── Write CSV row ────────────────────────────────────────────────────
        try
        {
            _writer.WriteLine(
                $"{_snap.seq}," +
                $"{_snap.cmd_seq_echo}," +
                $"{_snap.t0_echo}," +
                $"{_snap.t1_echo}," +
                $"{_snap.t3_recv}," +
                $"{_snap.tq_dequeue}," +
                $"{_snap.t4_physics}," +
                $"{_snap.t6_unity}," +
                $"{t7}," +
                $"{rtt}," +
                $"{processingMs}," +   // nanoseconds — full precision, no truncation
                $"{queueDelay}," +
                $"{network}," +
                $"{renderDelay}," +
                $"{mtp}," +
                $"{IsaacUR3Client.CurrentJitterMs:F4}");
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
