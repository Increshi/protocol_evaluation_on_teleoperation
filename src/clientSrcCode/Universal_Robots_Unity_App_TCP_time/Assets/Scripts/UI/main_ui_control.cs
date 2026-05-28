/****************************************************************************
MIT License
Copyright(c) 2020 Roman Parak
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*****************************************************************************
Author   : Roman Parak
Email    : Roman.Parak @outlook.com
Github   : https://github.com/rparak
File Name: main_ui_control.cs
****************************************************************************/

// System
using System;
using System.Text;
// Unity
using UnityEngine;
using UnityEngine.UI;
// TM
using TMPro;

public class main_ui_control : MonoBehaviour
{
    // -------------------- GameObjects -------------------- //
    public GameObject camera_obj;

    // -------------------- Panel Images -------------------- //
    // Drag the Image component of each panel root into these slots in the Inspector.
    public Image connection_panel_img;
    public Image diagnostic_panel_img;
    public Image joystick_panel_img;

    // -------------------- Connection-state indicator -------------------- //
    public Image connection_info_img;

    // -------------------- IP input -------------------- //
    public TMP_InputField ip_address_txt;

    // -------------------- Diagnostic text fields -------------------- //
    public TextMeshProUGUI position_x_txt,  position_y_txt,  position_z_txt;
    public TextMeshProUGUI position_rx_txt, position_ry_txt, position_rz_txt;
    public TextMeshProUGUI position_j1_txt, position_j2_txt, position_j3_txt;
    public TextMeshProUGUI position_j4_txt, position_j5_txt, position_j6_txt;
    public TextMeshProUGUI connectionInfo_txt;

    // -------------------- Internals -------------------- //
    private float ex_param = 100f;   // off-screen offset
    private UTF8Encoding utf8 = new UTF8Encoding();

    // ------------------------------------------------------------------------------------------------------------------------ //
    // ------------------------------------------------ INITIALIZATION {START} ------------------------------------------------ //
    // ------------------------------------------------------------------------------------------------------------------------ //
    void Start()
    {
        // Connection indicator — start red/disconnected
        if (connection_info_img != null) connection_info_img.color = new Color32(255, 0, 48, 50);
        if (connectionInfo_txt  != null) connectionInfo_txt.text = "Disconnect";

        // ---- Ensure all panels are interactable (positions unchanged from scene) ---- //
        if (connection_panel_img != null) connection_panel_img.raycastTarget = true;
        if (diagnostic_panel_img != null) diagnostic_panel_img.raycastTarget = true;
        if (joystick_panel_img   != null) joystick_panel_img.raycastTarget   = true;

        // ---- Reset diagnostic text ---- //
        if (position_x_txt  != null) position_x_txt.text  = "0.00";
        if (position_y_txt  != null) position_y_txt.text  = "0.00";
        if (position_z_txt  != null) position_z_txt.text  = "0.00";
        if (position_rx_txt != null) position_rx_txt.text = "0.00";
        if (position_ry_txt != null) position_ry_txt.text = "0.00";
        if (position_rz_txt != null) position_rz_txt.text = "0.00";
        if (position_j1_txt != null) position_j1_txt.text = "0.00";
        if (position_j2_txt != null) position_j2_txt.text = "0.00";
        if (position_j3_txt != null) position_j3_txt.text = "0.00";
        if (position_j4_txt != null) position_j4_txt.text = "0.00";
        if (position_j5_txt != null) position_j5_txt.text = "0.00";
        if (position_j6_txt != null) position_j6_txt.text = "0.00";

        // ---- Default IP ---- //
        if (ip_address_txt != null) ip_address_txt.text = "10.9.71.137";

        // ---- UR aux command init (keep for real-robot compatibility) ---- //
        ur_data_processing.UR_Control_Data.aux_command_str =
            "speedl([0.0,0.0,0.0,0.0,0.0,0.0], a = 0.15, t = 0.03)" + "\n";
        ur_data_processing.UR_Control_Data.command =
            utf8.GetBytes(ur_data_processing.UR_Control_Data.aux_command_str);

        // ---- Subscribe to TCP client connection events (indicator only) ---- //
        IsaacUR3Client.OnConnected    += OnIsaacConnected;
        IsaacUR3Client.OnDisconnected += OnIsaacDisconnected;
    }

    // -------------------- Helpers -------------------- //
    private void ShowPanel(Image img, float x, float y)
    {
        if (img == null) return;
        img.transform.localPosition = new Vector3(x, y, 0f);
        img.raycastTarget = true;
    }

    private void HidePanel(Image img, float offX)
    {
        if (img == null) return;
        img.transform.localPosition = new Vector3(offX + ex_param, 0f, 0f);
        img.raycastTarget = false;
    }

    // -------------------- Isaac TCP connection callbacks -------------------- //
    private void OnIsaacConnected()
    {
        if (connection_info_img != null) connection_info_img.color = new Color32(135, 255, 0, 50);
        if (connectionInfo_txt  != null) connectionInfo_txt.text = "Connect";
    }

    private void OnIsaacDisconnected()
    {
        if (connection_info_img != null) connection_info_img.color = new Color32(255, 0, 48, 50);
        if (connectionInfo_txt  != null) connectionInfo_txt.text = "Disconnect";
    }

    // -------------------- Unsubscribe on destroy -------------------- //
    void OnDestroy()
    {
        IsaacUR3Client.OnConnected    -= OnIsaacConnected;
        IsaacUR3Client.OnDisconnected -= OnIsaacDisconnected;
    }

    // ------------------------------------------------------------------------------------------------------------------------ //
    // ------------------------------------------------ MAIN FUNCTION {Cyclic} ------------------------------------------------ //
    // ------------------------------------------------------------------------------------------------------------------------ //
    void FixedUpdate()
    {
        // Keep UR data-processing structs in sync with the UI IP field
        if (ip_address_txt != null)
        {
            ur_data_processing.UR_Stream_Data.ip_address  = ip_address_txt.text;
            ur_data_processing.UR_Control_Data.ip_address = ip_address_txt.text;
        }

        // Diagnostic panel — only update if all text fields are assigned
        if (position_x_txt  == null || position_y_txt  == null || position_z_txt  == null ||
            position_rx_txt == null || position_ry_txt == null || position_rz_txt == null ||
            position_j1_txt == null || position_j2_txt == null || position_j3_txt == null ||
            position_j4_txt == null || position_j5_txt == null || position_j6_txt == null)
            return;

        // Cartesian position
        position_x_txt.text  = ((float)Math.Round(ur_data_processing.UR_Stream_Data.C_Position[0] * 1000f, 2)).ToString();
        position_y_txt.text  = ((float)Math.Round(ur_data_processing.UR_Stream_Data.C_Position[1] * 1000f, 2)).ToString();
        position_z_txt.text  = ((float)Math.Round(ur_data_processing.UR_Stream_Data.C_Position[2] * 1000f, 2)).ToString();
        // Rotation (euler, degrees)
        position_rx_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.C_Orientation[0] * (180 / Math.PI), 2)).ToString();
        position_ry_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.C_Orientation[1] * (180 / Math.PI), 2)).ToString();
        position_rz_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.C_Orientation[2] * (180 / Math.PI), 2)).ToString();
        // Joint angles (degrees)
        position_j1_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.J_Orientation[0] * (180 / Math.PI), 2)).ToString();
        position_j2_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.J_Orientation[1] * (180 / Math.PI), 2)).ToString();
        position_j3_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.J_Orientation[2] * (180 / Math.PI), 2)).ToString();
        position_j4_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.J_Orientation[3] * (180 / Math.PI), 2)).ToString();
        position_j5_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.J_Orientation[4] * (180 / Math.PI), 2)).ToString();
        position_j6_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.J_Orientation[5] * (180 / Math.PI), 2)).ToString();
    }

    // ------------------------------------------------------------------------------------------------------------------------ //
    // --------------------------------------------------- BUTTON HANDLERS ---------------------------------------------------- //
    // ------------------------------------------------------------------------------------------------------------------------ //

    // ---- Connection Panel ---- //
    public void TaskOnClick_ConnectionBTN()
    {
        ShowPanel(connection_panel_img, 0f, 0f);
    }
    public void TaskOnClick_EndConnectionBTN()
    {
        HidePanel(connection_panel_img, 1215f);
    }

    // ---- Diagnostic Panel ---- //
    public void TaskOnClick_DiagnosticBTN()
    {
        ShowPanel(diagnostic_panel_img, 0f, 0f);
    }
    public void TaskOnClick_EndDiagnosticBTN()
    {
        HidePanel(diagnostic_panel_img, 780f);
    }

    // ---- Joystick Panel ---- //
    public void TaskOnClick_JoystickBTN()
    {
        ShowPanel(joystick_panel_img, -265f, -129f);
    }
    public void TaskOnClick_EndJoystickBTN()
    {
        HidePanel(joystick_panel_img, 1550f);
    }

    // ---- Camera Presets ---- //
    public void TaskOnClick_CamViewRBTN()
    {
        camera_obj.transform.localPosition    = new Vector3(0.114f, 2.64f, -2.564f);
        camera_obj.transform.localEulerAngles = new Vector3(10f, -30f, 0f);
    }
    public void TaskOnClick_CamViewLBTN()
    {
        camera_obj.transform.localPosition    = new Vector3(-3.114f, 2.64f, -2.564f);
        camera_obj.transform.localEulerAngles = new Vector3(10f, 30f, 0f);
    }
    public void TaskOnClick_CamViewHBTN()
    {
        camera_obj.transform.localPosition    = new Vector3(-1.5f, 2.2f, -3.5f);
        camera_obj.transform.localEulerAngles = new Vector3(0f, 0f, 0f);
    }
    public void TaskOnClick_CamViewTBTN()
    {
        camera_obj.transform.localPosition    = new Vector3(-1.2f, 4f, 0f);
        camera_obj.transform.localEulerAngles = new Vector3(90f, 0f, 0f);
    }

    // ---- Connect / Disconnect ---- //
    public void TaskOnClick_ConnectBTN()
    {
        // Find and connect the TCP Isaac client
        IsaacUR3Client isaacClient = FindObjectOfType<IsaacUR3Client>();
        if (isaacClient != null)
        {
            // Push current IP from UI into the client
            if (isaacClient.ipInput != null)
                isaacClient.ipInput.text = ip_address_txt.text;
            else
                isaacClient.defaultIP = ip_address_txt.text;

            isaacClient.Connect();
        }
        else
        {
            Debug.LogWarning("IsaacUR3Client not found in scene!");
        }
    }

    public void TaskOnClick_DisconnectBTN()
    {
        IsaacUR3Client isaacClient = FindObjectOfType<IsaacUR3Client>();
        if (isaacClient != null)
            isaacClient.DisconnectFromServer();

        ur_data_processing.GlobalVariables_Main_Control.connect    = false;
        ur_data_processing.GlobalVariables_Main_Control.disconnect = true;
    }

    // ---- Lifecycle ---- //
    void OnApplicationQuit()
    {
        Destroy(this);
    }
}
