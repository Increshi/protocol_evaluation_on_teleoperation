// ------------------------------------------------------------------------------------------------------------------------ //
// ----------------------------------------------------- LIBRARIES -------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------ //

// -------------------- System -------------------- //
using System;
using System.Text;
// -------------------- Unity -------------------- //
using UnityEngine.EventSystems;
using UnityEngine;

// =============================================================================
// IsaacJogButton — jog button for Isaac Sim joint control over TCP
//
// T0 role: stamps DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() exactly once
// on OnPointerDown(). This is the "input captured" timestamp. It is passed to
// IsaacUR3Client.SendDelta() on every held frame and stored in
// IsaacUR3Client.T0_InputCapture — used by LatencyLogger for MTP = T7 - T0.
// T0 never travels to Isaac; it stays on the Unity side only.
// =============================================================================
public class IsaacJogButton : MonoBehaviour, IPointerDownHandler, IPointerUpHandler
{
    // -------------------- Params -------------------- //
    public float acceleration = 1.0f;
    public float time         = 0.05f;
    // speed_param as float[] — sign determines direction, magnitude is ignored
    // (direction is re-derived at Start from the sign of any non-zero entry)
    public float[] speed_param      = new float[6] { 0f, 0f, 0f, 0f, 0f, 0f };
    public float[] speed_param_null = new float[6] { 0f, 0f, 0f, 0f, 0f, 0f };
    // -------------------- Int -------------------- //
    public int index;

    // -------------------- Isaac Sim joint mapping -------------------- //
    // Remaps Cartesian-speedl button indices (0-11) → Isaac joint indices (0-5).
    // Buttons:  X+(0)→J0   X-(1)→J0   Y+(2)→J1   Y-(3)→J1
    //           Z+(4)→J2   Z-(5)→J2   RX+(6)→J3  RX-(7)→J3
    //           RY+(8)→J4  RY-(9)→J4  RZ+(10)→J5 RZ-(11)→J5
    private static readonly int[] _jointRemap = new int[]
    {
        0,  // index 0  → joint 0  (shoulder_pan)   xp_btn / rxm_btn
        0,  // index 1  → joint 0  (shoulder_pan)   xm_btn
        1,  // index 2  → joint 1  (shoulder_lift)  yp_btn
        1,  // index 3  → joint 1  (shoulder_lift)  ym_btn
        2,  // index 4  → joint 2  (elbow)          zp_btn
        2,  // index 5  → joint 2  (elbow)          zm_btn
        3,  // index 6  → joint 3  (wrist_1)        rxp_btn
        3,  // index 7  → joint 3  (wrist_1)        rxm_btn
        4,  // index 8  → joint 4  (wrist_2)        ryp_btn
        4,  // index 9  → joint 4  (wrist_2)        rym_btn
        5,  // index 10 → joint 5  (wrist_3)        rzp_btn
        5,  // index 11 → joint 5  (wrist_3)        rzm_btn
    };

    // Sign is derived from speed_param at Start: negative if any entry < 0, positive otherwise.
    [Tooltip("Magnitude of joint step per frame (radians). Sign is set automatically from speed_param.")]
    public float isaacDelta = 0.05f;

    // Resolved at Start: actual delta = |isaacDelta| * sign
    private float _resolvedDelta;

    // -------------------- UTF8Encoding (legacy UR command strings) -------------------- //
    private UTF8Encoding utf8 = new UTF8Encoding();

    private IsaacUR3Client _isaacClient;
    private bool _isHeld = false;

    // T0: input-capture timestamp — UTC ms, stamped ONCE on pointer-down.
    // Same clock as T1 (DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()).
    private long _t0 = 0;

    void Start()
    {
        _isaacClient = FindObjectOfType<IsaacUR3Client>();

        // Derive direction sign from speed_param
        float sign = 1f;
        foreach (float v in speed_param)
        {
            if (v < 0f) { sign = -1f; break; }
        }
        _resolvedDelta = Mathf.Abs(isaacDelta) * sign;
    }

    void Update()
    {
        // While held, stream delta every frame — T0 stays fixed at the original press time
        if (_isHeld && _isaacClient != null)
        {
            int jointIndex = (index >= 0 && index < _jointRemap.Length) ? _jointRemap[index] : 0;
            // Pass _t0 → SendDelta writes it to IsaacUR3Client.T0_InputCapture
            // SendDelta stamps T1 internally just before stream.Write()
            _isaacClient.SendDelta(jointIndex, _resolvedDelta, _t0);
        }
    }

    // -------------------- Button -> Pressed -------------------- //
    public void OnPointerDown(PointerEventData eventData)
    {
        // T0: exact moment user input was captured — UTC ms, same clock as T1 and T6
        _t0 = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

        // Legacy real-robot speedl command string
        ur_data_processing.UR_Control_Data.aux_command_str =
            string.Format(System.Globalization.CultureInfo.InvariantCulture,
                "speedl([{0},{1},{2},{3},{4},{5}], a={6}, t={7})\n",
                speed_param[0], speed_param[1], speed_param[2],
                speed_param[3], speed_param[4], speed_param[5],
                acceleration, time);
        ur_data_processing.UR_Control_Data.command =
            utf8.GetBytes(ur_data_processing.UR_Control_Data.aux_command_str);
        ur_data_processing.UR_Control_Data.button_pressed[index] = true;

        // Isaac Sim — start continuous delta stream
        _isHeld = true;
    }

    // -------------------- Button -> Un-Pressed -------------------- //
    public void OnPointerUp(PointerEventData eventData)
    {
        ur_data_processing.UR_Control_Data.button_pressed[index] = false;
        _isHeld = false;
        _t0 = 0;
    }
}
