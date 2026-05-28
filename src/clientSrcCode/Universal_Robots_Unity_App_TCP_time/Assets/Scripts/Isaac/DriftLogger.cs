using UnityEngine;
using System.IO;
using System;

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

        // Use the echoed cmd_seq to gate logging (same pattern as LatencyLogger)
        int cmdSeq = IsaacUR3Client.LastReceivedCmdSeq;
        if (cmdSeq <= 0) return;
        if (cmdSeq == _lastLoggedCmdSeq) return;
        _lastLoggedCmdSeq = cmdSeq;

        if (_startTimeMs == -1) _startTimeMs = IsaacUR3Client.T6_UnityReceived;
        double timeSec = (IsaacUR3Client.T6_UnityReceived - _startTimeMs) / 1000.0;

        if (IsaacUR3Client.CmdToExpected.TryGetValue(cmdSeq, out float[] expectedArray))
        {
            float expected = expectedArray[trackedJointIndex];
            float actual = IsaacUR3Client.JointAnglesRad[trackedJointIndex];

            IsaacUR3Client.CmdToExpected.TryRemove(cmdSeq, out _);

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
