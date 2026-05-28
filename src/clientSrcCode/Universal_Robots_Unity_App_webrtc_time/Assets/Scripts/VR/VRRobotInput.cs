using UnityEngine;
using UnityEngine.XR;
using System;

public class VRRobotInput : MonoBehaviour
{
    public IsaacUR3WebRTCClient client; // Drag IsaacManager here
    public float speed = 0.05f;

    [Tooltip("Joystick deadzone threshold. Axis must exceed this to count as active.")]
    public float deadzone = 0.1f;

    [Tooltip("Minimum milliseconds between sends while the joystick is held. 0 = rising-edge only (no repeats).")]
    public long sendIntervalMs = 20;

    // Edge-detection: SendDelta fires on the rising edge (inactive→active).
    // While held, it repeats at most once per sendIntervalMs.
    // sendIntervalMs=0 disables repeats entirely (single-tap behaviour).
    private bool _wasActiveX  = false;
    private bool _wasActiveY  = false;
    private long _lastSendXMs = 0;
    private long _lastSendYMs = 0;

    void Update()
    {
        // 1. Get the Left Controller
        var leftHandDevices = new System.Collections.Generic.List<InputDevice>();
        InputDevices.GetDevicesAtXRNode(XRNode.LeftHand, leftHandDevices);

        if (leftHandDevices.Count == 1)
        {
            InputDevice device = leftHandDevices[0];
            Vector2 joystickValue;

            // 2. Read the Joystick (Primary 2D Axis)
            if (device.TryGetFeatureValue(CommonUsages.primary2DAxis, out joystickValue))
            {
                bool activeX = Mathf.Abs(joystickValue.x) > deadzone;
                bool activeY = Mathf.Abs(joystickValue.y) > deadzone;
                long now     = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

                // Move Robot Base (Joint 0)
                // Rising edge: always send immediately.
                // Held: repeat only if sendIntervalMs > 0 and interval has elapsed.
                if (activeX)
                {
                    bool risingEdge = !_wasActiveX;
                    bool intervalOk = sendIntervalMs > 0 && (now - _lastSendXMs) >= sendIntervalMs;
                    if (risingEdge || intervalOk)
                    {
                        long t0 = risingEdge ? now : _lastSendXMs; // T0 = moment of original press on edge, else last send
                        client.SendDelta(0, joystickValue.x * speed, t0);
                        _lastSendXMs = now;
                    }
                }

                // Move Shoulder (Joint 1)
                if (activeY)
                {
                    bool risingEdge = !_wasActiveY;
                    bool intervalOk = sendIntervalMs > 0 && (now - _lastSendYMs) >= sendIntervalMs;
                    if (risingEdge || intervalOk)
                    {
                        long t0 = risingEdge ? now : _lastSendYMs;
                        client.SendDelta(1, -joystickValue.y * speed, t0);
                        _lastSendYMs = now;
                    }
                }

                _wasActiveX = activeX;
                _wasActiveY = activeY;
            }
            else
            {
                // Device lost — reset all state
                _wasActiveX  = false;
                _wasActiveY  = false;
                _lastSendXMs = 0;
                _lastSendYMs = 0;
            }
        }
    }
}
