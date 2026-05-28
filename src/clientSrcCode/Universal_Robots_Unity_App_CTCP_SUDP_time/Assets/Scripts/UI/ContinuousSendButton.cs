using UnityEngine;

public class ContinuousSendButton : MonoBehaviour
{
    [Header("Configuration")]
    public IsaacUR3Client client;
    public int jointIndex = 0;
    public float deltaAmount = 0.05f;

    private bool _isSending = false;

    // Wire this to the Button's OnClick.
    public void ToggleContinuousSend()
    {
        if (client == null) return;

        if (_isSending)
        {
            client.StopContinuousDelta();
            _isSending = false;
        }
        else
        {
            client.StartContinuousDelta(jointIndex, deltaAmount);
            _isSending = true;
        }
    }

    private void OnDisable()
    {
        if (_isSending && client != null)
        {
            client.StopContinuousDelta();
            _isSending = false;
        }
    }
}
