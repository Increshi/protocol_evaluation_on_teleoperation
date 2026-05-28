using System;
using UnityEngine;

public class ContinuousSendButton : MonoBehaviour
{
    [Header("Configuration")]
    public IsaacUR3Client client;
    public int jointIndex = 0;
    public float deltaAmount = 0.05f;

    private bool _isSending = false;
    private long _t0 = 0;

    // Wire this to the Button's OnClick.
    public void ToggleContinuousSend()
    {
        if (client == null) return;

        if (_isSending)
        {
            _isSending = false;
            StopSendLoop();
        }
        else
        {
            _isSending = true;
            _t0 = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            StartSendLoop();
        }
    }

    private void OnDisable()
    {
        if (_isSending && client != null)
        {
            _isSending = false;
            StopSendLoop();
        }
    }

    private void Update()
    {
        if (_isSending && client != null)
        {
            client.SendDelta(jointIndex, deltaAmount, _t0);
        }
    }

    private void StartSendLoop()
    {
        // Reserved for symmetry with StopSendLoop; no coroutine used for per-frame send.
    }

    private void StopSendLoop()
    {
        _t0 = 0;
    }
}
