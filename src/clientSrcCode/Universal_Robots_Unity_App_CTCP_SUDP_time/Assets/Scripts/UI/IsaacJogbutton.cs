using UnityEngine;
using UnityEngine.EventSystems;
using System;

public class IsaacJogButton : MonoBehaviour, IPointerDownHandler, IPointerUpHandler
{
    [Header("Configuration")]
    public IsaacUR3Client client; // Drag IsaacManager here
    public int jointIndex;        // 0=Base, 1=Shoulder, 2=Elbow...
    public float speed = 0.05f;   // Positive for +, Negative for -

    private bool isPressed = false;

    public void OnPointerDown(PointerEventData eventData)
    {
        isPressed = true;
    }

    public void OnPointerUp(PointerEventData eventData)
    {
        isPressed = false;
    }

    void FixedUpdate()
    {
        if (isPressed && client != null)
        {
            // T0: input captured — passed into SendDelta for packet embedding
            long t0 = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            // Send incremental move command continuously while holding
            client.SendDelta(jointIndex, speed, t0);
        }
    }
}
