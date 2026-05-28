using UnityEngine;
using System.IO;
using System;
using System.Collections.Concurrent;

public class DriftLogger : MonoBehaviour
{
    [Header("Logging Settings")]
    [Tooltip("Enable / disable CSV writing at runtime.")]
    public bool enableLogging = true;
    
    [Tooltip("The joint index (0-5) you are moving with ContinuousSendButton")]
    public int trackedJointIndex = 0;

    private StreamWriter _writer;
    private string _logPath;
    private int _lastLoggedCmdSeq = -1;
    private long _startTimeMs = -1;

    void Start()
    {
        if (!enableLogging) return;

        string timestamp = DateTime.UtcNow.ToString("yyyyMMdd_HHmmss");
        _logPath = Path.Combine(Application.persistentDataPath, $"drift_log_{timestamp}.csv");
        Debug.Log($"[DriftLogger] Writing drift analytics to: {_logPath}");

        try
        {
            _writer = new StreamWriter(new FileStream(_logPath, FileMode.Create, FileAccess.Write, FileShare.Read));
            _writer.AutoFlush = true;
            
            // Strict 3-column CSV format expected by plot_drift.py
            _writer.WriteLine("Time_sec,ExpectedPos,ActualPos");
        }
        catch (Exception e)
        {
            Debug.LogError($"[DriftLogger] Failed to open drift log file: {e.Message}");
            enableLogging = false;
        }
    }

    void LateUpdate()
    {
        if (!enableLogging || _writer == null) return;

        // Safely fetch latest state from WebRTC client
        IsaacUR3WebRTCClient.IsaacState snap;
        lock (IsaacUR3WebRTCClient.StateLock)
        {
            if (IsaacUR3WebRTCClient.LatestState == null) return;
            snap = IsaacUR3WebRTCClient.LatestState;
        }

        // Only log when we receive confirmation that a completely new command was processed by Isaac Sim
        if (snap.cmd_seq_echo <= 0) return;
        if (snap.cmd_seq_echo == _lastLoggedCmdSeq) return;
        _lastLoggedCmdSeq = snap.cmd_seq_echo;

        // Establish relative starting time for graphing purposes (start at t=0s)
        if (_startTimeMs == -1) _startTimeMs = snap.t6_unity;
        double timeSec = (snap.t6_unity - _startTimeMs) / 1000.0;

        // Look up the expected position mapped to this exact command
        if (IsaacUR3WebRTCClient.CmdToExpected.TryGetValue(snap.cmd_seq_echo, out float[] expectedArray))
        {
            float expected = expectedArray[trackedJointIndex];
            
            // The actual resolved position the server reported for this state packet
            float actual = IsaacUR3WebRTCClient.JointAnglesRad[trackedJointIndex];

            // Free up memory so the tracking dictionary doesn't infinitely expand
            IsaacUR3WebRTCClient.CmdToExpected.TryRemove(snap.cmd_seq_echo, out _);

            try
            {
                _writer.WriteLine($"{timeSec:F4},{expected:F4},{actual:F4}");
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[DriftLogger] Write error: {e.Message}");
            }
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
