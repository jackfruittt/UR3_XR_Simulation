// Author: Jackson Russell

using System.Text;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UIElements;

// World-space wrist panel parented to the left controller.
// Builds at runtime: RenderTexture -> cloned PanelSettings -> UIDocument -> textured quad.
// Left Y toggles visibility. While open, the pilot HUD hides to avoid duplicate data.
// F5 is a keyboard fallback for testing in the Editor without a headset.
//
// Setup: assign wristUxml (Assets/UI/WristHUD.uxml) and sourcePanelSettings
// (Assets/UI Toolkit/PanelSettings.asset) in the Inspector. Everything else autowires.
public class WristHUDController : MonoBehaviour
{

    [Header("UXML / Theme")]
    [Tooltip("Assign Assets/UI/WristHUD.uxml")]
    public VisualTreeAsset wristUxml;

    [Tooltip("Assign the existing scene PanelSettings asset (Assets/UI Toolkit/PanelSettings.asset). " +
             "It will be cloned at runtime and its targetTexture pointed at the wrist RT.")]
    public PanelSettings sourcePanelSettings;

    [Header("Dependencies")]
    public HUDController           hudController;
    public EEFTargetController     target;
    public JacobianIKSolver        ikSolver;
    public RobotFKSolver           fkSolver;
    public UR3SourceDestinationPublisher publisher;
    public HandEyeCalibrationCollector   collector;

    [Header("Quad Geometry")]
    [Tooltip("Width of the wrist panel quad in metres.")]
    public float quadWidth  = 0.28f;
    [Tooltip("Height of the wrist panel quad in metres.")]
    public float quadHeight = 0.37f;
    [Tooltip("Local position offset from Left Controller origin.")]
    public Vector3 quadLocalPos    = new Vector3(0f, 0.12f, 0f);
    [Tooltip("Local Euler rotation so it faces up toward the wearer.")]
    public Vector3 quadLocalEuler  = new Vector3(-45f, 0f, 0f);

    [Header("Render Texture")]
    public int rtWidth   = 600;
    public int rtHeight  = 800;

    RenderTexture    _rt;
    PanelSettings    _panelSettings;
    GameObject       _quadGO;
    UIDocument       _uiDoc;
    VisualElement    _root;
    bool             _wristVisible;
    bool             _initialised;

    // Tab
    int              _activeTab;
    VisualElement    _panelRobot;
    VisualElement    _panelCalib;
    Button           _tabRobot;
    Button           _tabCalib;

    // Robot panel labels
    Label _valEef, _valTarget, _valError;
    Label _valIkMode, _valManip, _valVel, _valState;
    Label _valJointsAbc, _valJointsDef;
    Label _valCamFps, _valCamRes, _valCamK;

    // Calib panel
    Label  _valCalibStatus;
    Button _btnAuto, _btnManual, _btnConfirm, _btnFinish, _btnCancel, _btnLoad;
    Label  _valPairs, _valResidual, _valCalibT;

    // XR input
    InputAction _xrWristToggle;   // Left Y
    InputAction _xrTabRobot;      // Left X (while wrist visible)
    InputAction _xrTabCalib;      // Left thumbstick click (while wrist visible)

    readonly float[]       _jointBuf = new float[6];
    readonly StringBuilder _sb       = new StringBuilder(256);


    void Awake()
    {
        if (hudController  == null) hudController  = FindObjectOfType<HUDController>();
        if (target         == null) target         = FindObjectOfType<EEFTargetController>();
        if (ikSolver       == null) ikSolver       = FindObjectOfType<JacobianIKSolver>();
        if (fkSolver       == null) fkSolver       = FindObjectOfType<RobotFKSolver>();
        if (publisher      == null) publisher      = FindObjectOfType<UR3SourceDestinationPublisher>();
        if (collector      == null) collector      = FindObjectOfType<HandEyeCalibrationCollector>();
    }

    void Start()
    {
        // Wait for XRRigSetup to finish (it yields one frame).
        StartCoroutine(InitAfterRig());
    }

    System.Collections.IEnumerator InitAfterRig()
    {
        yield return null;   // let XRRigSetup.Start() complete first
        yield return null;

        if (!BuildWristPanel())
        {
            Debug.LogWarning("[WristHUDController] Could not build wrist panel. " +
                             "Assign wristUxml and ensure XR Origin rig is in the scene.");
            yield break;
        }

        // UIDocument populates rootVisualElement one frame after creation.
        yield return null;
        yield return null;

        BindElements();
        InitXRActions();

        if (collector != null)
        {
            collector.OnReadyToCapture      += OnReadyToCapture;
            collector.OnCalibrationComplete += OnCalibrationComplete;
        }

        _wristVisible  = false;
        _quadGO.SetActive(false);   // hidden by default
        _initialised   = true;
    }

    void OnDisable()
    {
        if (collector != null)
        {
            collector.OnReadyToCapture      -= OnReadyToCapture;
            collector.OnCalibrationComplete -= OnCalibrationComplete;
        }
        _xrWristToggle?.Dispose();
        _xrTabRobot?.Dispose();
        _xrTabCalib?.Dispose();
    }

    void Update()
    {
        if (!_initialised) return;

        HandleInput();
        if (!_wristVisible) return;

        switch (_activeTab)
        {
            case 0: UpdateRobotPanel();  break;
            case 1: UpdateCalibPanel();  break;
        }
    }


    bool BuildWristPanel()
    {
        if (wristUxml == null)
        {
            Debug.LogError("[WristHUDController] wristUxml not assigned.");
            return false;
        }

        var rig = XRRigSetup.Instance;
        if (rig == null || rig.LeftController == null)
        {
            Debug.LogError("[WristHUDController] XRRigSetup.Instance.LeftController is null. " +
                           "Ensure XRRigSetup is in the scene and the XR rig is configured.");
            return false;
        }

        // 1 ── RenderTexture
        _rt = new RenderTexture(rtWidth, rtHeight, 0, RenderTextureFormat.ARGB32)
        {
            name         = "WristHUD_RT",
            filterMode   = FilterMode.Bilinear,
            antiAliasing = 1,
        };
        _rt.Create();

        // Clone the existing PanelSettings to inherit theme and scale settings,
        // then redirect targetTexture to the new RT.
        if (sourcePanelSettings == null)
        {
            Debug.LogError("[WristHUDController] sourcePanelSettings not assigned. " +
                           "Drag Assets/UI Toolkit/PanelSettings.asset into the Source Panel Settings field.");
            return false;
        }
        _panelSettings = Object.Instantiate(sourcePanelSettings);
        _panelSettings.targetTexture = _rt;
        _panelSettings.clearColor    = true;
        _panelSettings.colorClearValue = new Color(0f, 0f, 0f, 0f);

        Debug.Log($"[WristHUDController] PanelSettings cloned. targetTexture={_rt.name} {_rt.width}x{_rt.height}");

        // 3 ── Host GO + UIDocument
        var hostGO = new GameObject("WristHUDDocument");
        _uiDoc                   = hostGO.AddComponent<UIDocument>();
        _uiDoc.panelSettings     = _panelSettings;
        _uiDoc.visualTreeAsset   = wristUxml;
        hostGO.transform.SetParent(rig.LeftController, false);
        hostGO.transform.localPosition = Vector3.zero;

        // 4 ── Quad
        _quadGO = GameObject.CreatePrimitive(PrimitiveType.Quad);
        _quadGO.name = "WristHUDQuad";
        Destroy(_quadGO.GetComponent<MeshCollider>());

        _quadGO.transform.SetParent(rig.LeftController, false);
        _quadGO.transform.localPosition    = quadLocalPos;
        _quadGO.transform.localEulerAngles = quadLocalEuler;
        _quadGO.transform.localScale       = new Vector3(quadWidth, quadHeight, 1f);

        // Sprites/Default handles transparency and is available in both Built-in and URP.
        var shader = Shader.Find("Sprites/Default");
        if (shader == null) shader = Shader.Find("Unlit/Transparent");
        var mat = new Material(shader);
        mat.mainTexture = _rt;
        _quadGO.GetComponent<Renderer>().material = mat;

        Debug.Log("[WristHUDController] Wrist panel built on Left Controller.");
        return true;
    }


    void BindElements()
    {
        _root = _uiDoc.rootVisualElement;
        Debug.Log($"[WristHUDController] BindElements: root={(object)_root ?? (object)"null"}  " +
                  $"childCount={_root?.childCount}  visualTreeAsset={_uiDoc.visualTreeAsset?.name}");
        if (_root == null || _root.childCount == 0)
        {
            Debug.LogError("[WristHUDController] rootVisualElement empty - wristUxml not assigned or UXML failed to load.");
            return;
        }

        _panelRobot = _root.Q<VisualElement>("panel-robot");
        _panelCalib = _root.Q<VisualElement>("panel-calib");
        _tabRobot   = _root.Q<Button>("tab-robot");
        _tabCalib   = _root.Q<Button>("tab-calib");

        if (_tabRobot != null) _tabRobot.clicked += () => SetTab(0);
        if (_tabCalib != null) _tabCalib.clicked += () => SetTab(1);

        _valEef       = _root.Q<Label>("val-eef");
        _valTarget    = _root.Q<Label>("val-target");
        _valError     = _root.Q<Label>("val-error");
        _valIkMode    = _root.Q<Label>("val-ik-mode");
        _valManip     = _root.Q<Label>("val-manip");
        _valVel       = _root.Q<Label>("val-vel");
        _valState     = _root.Q<Label>("val-state");
        _valJointsAbc = _root.Q<Label>("val-joints-abc");
        _valJointsDef = _root.Q<Label>("val-joints-def");
        _valCamFps    = _root.Q<Label>("val-cam-fps");
        _valCamRes    = _root.Q<Label>("val-cam-res");
        _valCamK      = _root.Q<Label>("val-cam-k");

        _valCalibStatus = _root.Q<Label>("val-calib-status");
        _btnAuto        = _root.Q<Button>("btn-auto");
        _btnManual      = _root.Q<Button>("btn-manual");
        _btnConfirm     = _root.Q<Button>("btn-confirm");
        _btnFinish      = _root.Q<Button>("btn-finish");
        _btnCancel      = _root.Q<Button>("btn-cancel");
        _btnLoad        = _root.Q<Button>("btn-load");
        _valPairs       = _root.Q<Label>("val-pairs");
        _valResidual    = _root.Q<Label>("val-residual");
        _valCalibT      = _root.Q<Label>("val-calib-t");

        if (_btnAuto    != null) _btnAuto.clicked    += OnAutoClicked;
        if (_btnManual  != null) _btnManual.clicked  += OnManualClicked;
        if (_btnConfirm != null) _btnConfirm.clicked += OnConfirmClicked;
        if (_btnFinish  != null) _btnFinish.clicked  += OnFinishClicked;
        if (_btnCancel  != null) _btnCancel.clicked  += OnCancelClicked;
        if (_btnLoad    != null) _btnLoad.clicked    += OnLoadClicked;

        SetTab(0);

        if (CalibrationResult.Exists())
            SetCalibStatus("Calibration on disk", false);
        else
            SetCalibStatus("No calibration saved", false);
    }

    void InitXRActions()
    {
        // Left Y - toggle wrist panel
        _xrWristToggle = new InputAction("WristToggle", InputActionType.Button,
            binding: "<XRController>{LeftHand}/secondaryButton");

        // Left X - Robot tab (only active while wrist is open)
        _xrTabRobot = new InputAction("WristTabRobot", InputActionType.Button,
            binding: "<XRController>{LeftHand}/primaryButton");

        // Thumbstick click - Calib tab
        _xrTabCalib = new InputAction("WristTabCalib", InputActionType.Button,
            binding: "<XRController>{LeftHand}/thumbstickClicked");

        _xrWristToggle.Enable();
        _xrTabRobot.Enable();
        _xrTabCalib.Enable();
    }

    void HandleInput()
    {
        if (_xrWristToggle != null && _xrWristToggle.WasPressedThisFrame())
            SetWristVisible(!_wristVisible);

        // F5 keyboard fallback for Editor testing
        if (Keyboard.current != null && Keyboard.current.f5Key.wasPressedThisFrame)
            SetWristVisible(!_wristVisible);

        if (!_wristVisible) return;

        // Tab switching while wrist is open
        if (_xrTabRobot != null && _xrTabRobot.WasPressedThisFrame()) SetTab(0);
        if (_xrTabCalib != null && _xrTabCalib.WasPressedThisFrame()) SetTab(1);

        if (Keyboard.current != null)
        {
            if (Keyboard.current.f1Key.wasPressedThisFrame) SetTab(0);
            if (Keyboard.current.f2Key.wasPressedThisFrame) SetTab(1);
        }
    }

    void SetWristVisible(bool visible)
    {
        _wristVisible = visible;
        _quadGO.SetActive(visible);

        // While wrist is open, hide the pilot HUD to avoid duplicate data.
        if (hudController != null)
            hudController.SetPilotVisible(!visible);
    }

    void SetTab(int index)
    {
        _activeTab = index;
        _panelRobot?.EnableInClassList("panel-hidden", index != 0);
        _panelCalib?.EnableInClassList("panel-hidden", index != 1);
        _tabRobot?.EnableInClassList("tab-active", index == 0);
        _tabCalib?.EnableInClassList("tab-active", index == 1);
    }


    void UpdateRobotPanel()
    {
        if (target == null) return;

        Vector3 eefWorld = fkSolver != null ? fkSolver.GetEEFPosition() : Vector3.zero;
        Vector3 base3    = target.robotBase != null ? target.robotBase.position : Vector3.zero;
        Vector3 eefLocal = eefWorld - base3;

        SetText(_valEef, $"{eefLocal.x:+0.000;-0.000}  {eefLocal.y:+0.000;-0.000}  {eefLocal.z:+0.000;-0.000} m");

        Vector3 tgtLocal = target.TargetPosition - base3;
        SetText(_valTarget, $"{tgtLocal.x:+0.000;-0.000}  {tgtLocal.y:+0.000;-0.000}  {tgtLocal.z:+0.000;-0.000} m");
        SetColor(_valTarget, target.IsControllingTarget ? "#FFA050"
                           : target.IsInCrosshairRange  ? "#FFD020" : "#E6E6F0");

        float errM = (target.TargetPosition - eefWorld).magnitude;
        SetText(_valError, $"{errM * 1000f:F1} mm   ring {target.TableNormalisedRadius() * 100f:F0}%");
        SetColor(_valError, errM < 0.005f ? "#3CDD64" : errM < 0.025f ? "#FFD932" : "#FF5046");

        if (ikSolver != null)
        {
            float  beta    = ikSolver.LastBlendFactor;
            float  manip   = ikSolver.LastManipulability;
            string modeStr = beta > 0.5f ? "DLS" : "GD";
            SetText(_valIkMode, $"{modeStr}   beta {beta:F2}");
            SetColor(_valIkMode, ColorLerpHex(new Color(1f, 0.38f, 0.26f), new Color(0.30f, 0.92f, 0.42f), beta));

            SetText(_valManip, manip.ToString("F4"));

            float[] vels = ikSolver.LastJointVelocitiesRad;
            float   peak = 0f;
            if (vels != null)
                foreach (float v in vels)
                    if (Mathf.Abs(v) > peak) peak = Mathf.Abs(v);
            float peakPct = peak / Mathf.PI * 100f;
            SetText(_valVel, $"{peakPct:F0}%");
            SetColor(_valVel, ColorLerpHex(new Color(0.30f, 0.92f, 0.42f), new Color(1f, 0.38f, 0.26f),
                                           Mathf.Clamp01(peakPct / 100f)));

            var ss = ikSolver.CurrentSolverState;
            string stateStr, stateCol;
            if      (ss == JacobianIKSolver.SolverState.EscapingToHome)
            { stateStr = "HOMING";   stateCol = "#FFAA22"; }
            else if (ss == JacobianIKSolver.SolverState.ResumeDelay)
            { stateStr = "RESUMING"; stateCol = "#FFE633"; }
            else if (target.IsControllingTarget)
            { stateStr = "CONTROL";  stateCol = "#FFA050"; }
            else if (target.IsInCrosshairRange)
            { stateStr = "AIMED";    stateCol = "#FFD020"; }
            else
            { stateStr = "IDLE";     stateCol = "#888898"; }
            SetText(_valState, stateStr);
            SetColor(_valState, stateCol);
        }

        if (publisher != null && publisher.GetActualJointAnglesInto(_jointBuf))
        {
            SetText(_valJointsAbc, $"{_jointBuf[0]:F1}  {_jointBuf[1]:F1}  {_jointBuf[2]:F1}");
            SetText(_valJointsDef, $"{_jointBuf[3]:F1}  {_jointBuf[4]:F1}  {_jointBuf[5]:F1}");
        }
    }


    void UpdateCalibPanel()
    {
        if (collector == null) return;

        var  state     = collector.CurrentState;
        bool running   = state != HandEyeCalibrationCollector.State.Idle
                      && state != HandEyeCalibrationCollector.State.Done
                      && state != HandEyeCalibrationCollector.State.Failed;
        bool canFinish = running && collector.ManualMode
                      && collector.CapturedCount >= HandEyeSolver.MinPairs;
        bool readyToConfirm = state == HandEyeCalibrationCollector.State.ReadyToCapture;

        SetEnabled(_btnAuto,     !running);
        SetEnabled(_btnManual,   !running);
        SetEnabled(_btnLoad,     !running);
        SetEnabled(_btnCancel,    running);
        SetEnabled(_btnConfirm,   readyToConfirm);
        SetEnabled(_btnFinish,    canFinish);

        SetText(_valPairs, collector.CapturedCount.ToString());

        if (!float.IsNaN(collector.ResidualDeg))
        {
            SetText(_valResidual, $"{collector.ResidualDeg:F2} deg");
            SetColor(_valResidual, collector.ResidualDeg < 2f ? "#3CDD64"
                                 : collector.ResidualDeg < 5f ? "#FFD932"
                                 :                              "#FF5046");
        }
        else SetText(_valResidual, "--");
    }

    void SetCalibStatus(string msg, bool ok)
    {
        SetText(_valCalibStatus, msg);
        SetColor(_valCalibStatus, ok ? "#3CDD64" : "#E6E6F0");
    }


    void OnAutoClicked()
    {
        if (collector == null) return;
        collector.SetManualMode(false);
        collector.StartCalibration();
        SetCalibStatus("Auto session started...", false);
    }

    void OnManualClicked()
    {
        if (collector == null) return;
        collector.SetManualMode(true);
        collector.StartCalibration();
        SetCalibStatus("Manual session started - move robot, then Confirm", false);
    }

    void OnConfirmClicked()  => collector?.ConfirmCapture();
    void OnFinishClicked()   => collector?.FinishManualSession();

    void OnCancelClicked()
    {
        collector?.CancelCalibration();
        SetCalibStatus("Cancelled", false);
    }

    void OnLoadClicked()
    {
        var result = CalibrationResult.Load();
        if (result == null) { SetCalibStatus("No calibration file found", false); return; }
        collector?.ApplyCalibrationToScene(result);
        Vector3 t = result.Translation;
        SetCalibStatus(
            $"Loaded  residual {result.residualDeg:F2} deg  t ({t.x:+0.00;-0.00}, {t.y:+0.00;-0.00}, {t.z:+0.00;-0.00})",
            result.residualDeg < 5f);
        SetText(_valResidual, $"{result.residualDeg:F2} deg");
        SetText(_valCalibT,   $"{t.x:+0.000;-0.000}  {t.y:+0.000;-0.000}  {t.z:+0.000;-0.000} m");
    }

    void OnReadyToCapture()
    {
        SetCalibStatus($"Pose {collector.CurrentPose + 1} ready - press Confirm", false);
    }

    void OnCalibrationComplete(CalibrationResult result)
    {
        if (result == null) { SetCalibStatus("Calibration failed - not enough pairs", false); return; }
        Vector3 t = result.Translation;
        SetCalibStatus($"Done  residual {result.residualDeg:F2} deg  pairs {result.pairsUsed}", result.residualDeg < 5f);
        SetText(_valResidual, $"{result.residualDeg:F2} deg");
        SetText(_valCalibT,   $"{t.x:+0.000;-0.000}  {t.y:+0.000;-0.000}  {t.z:+0.000;-0.000} m");
        SetText(_valPairs,    result.pairsUsed.ToString());
    }


    static void SetText(Label lbl, string text)
    {
        if (lbl != null && lbl.text != text) lbl.text = text;
    }

    static void SetColor(Label lbl, string hex)
    {
        if (lbl == null) return;
        if (ColorUtility.TryParseHtmlString(hex, out Color c))
            lbl.style.color = c;
    }

    static void SetEnabled(Button btn, bool enabled)
    {
        if (btn == null) return;
        btn.SetEnabled(enabled);
        btn.EnableInClassList("btn-disabled", !enabled);
    }

    static string ColorLerpHex(Color a, Color b, float t)
        => "#" + ColorUtility.ToHtmlStringRGB(Color.Lerp(a, b, t));
}
