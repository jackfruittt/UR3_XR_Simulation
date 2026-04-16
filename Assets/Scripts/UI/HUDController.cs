// Author: Jackson Russell

using System.Text;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UIElements;

public class HUDController : MonoBehaviour
{
    public static HUDController Instance { get; private set; }
    [Header("UI Document")]
    [SerializeField] UIDocument _uiDocument;

    [Header("Dependencies")]
    [SerializeField] EEFTargetController          _target;
    [SerializeField] JacobianIKSolver             _ikSolver;
    [SerializeField] RobotFKSolver                _fkSolver;
    [SerializeField] UR3SourceDestinationPublisher _publisher;
    [SerializeField] Detection                    _detection;
    [SerializeField] ROSPointCloudRenderer        _renderer;
    [SerializeField] SimpleImageSubscriber        _colourSource;
    [SerializeField] HandEyeCalibrationCollector  _collector;

    // Tab panels
    VisualElement _panelRobot;
    VisualElement _panelCalib;
    Button        _tabRobot;
    Button        _tabCalib;
    int           _activeTab = 0;

    // Robot panel
    Label _valEef;
    Label _valTarget;
    Label _valError;
    Label _valIkMode;
    Label _valManip;
    Label _valVel;
    Label _valState;
    Label _valJointsAbc;
    Label _valJointsDef;
    Label _valCamFps;
    Label _valCamRes;
    Label _valCamK;


    // Calibration panel
    Label   _valCalibStatus;
    Button  _btnAuto;
    Button  _btnManual;
    Button  _btnConfirm;
    Button  _btnFinish;
    Button  _btnCancel;
    Button  _btnLoad;
    Label   _valPairs;
    Label   _valResidual;
    Label   _valCalibT;

    bool _hudVisible = true;

    // True while the mouse pointer is inside the HUD panel.
    // Read by FirstPersonCamera to suppress cursor re-lock on UI clicks.
    public static bool IsPointerOver { get; private set; }

    // XR input actions for the pilot HUD.
    // Left Y is reserved for WristHUDController.
    private InputAction _xrTabRobot;   // left X - Robot tab
    private InputAction _xrHudToggle;  // left thumbstick click - toggle pilot HUD

    readonly float[]  _jointBuf = new float[6];
    readonly StringBuilder _sb  = new StringBuilder(256);

    void Awake()
    {
        Instance = this;
        if (_target      == null) _target      = FindObjectOfType<EEFTargetController>();
        if (_ikSolver    == null) _ikSolver    = FindObjectOfType<JacobianIKSolver>();
        if (_fkSolver    == null) _fkSolver    = FindObjectOfType<RobotFKSolver>();
        if (_publisher   == null) _publisher   = FindObjectOfType<UR3SourceDestinationPublisher>();
        if (_detection   == null) _detection   = FindObjectOfType<Detection>();
        if (_renderer    == null) _renderer    = FindObjectOfType<ROSPointCloudRenderer>();
        if (_colourSource == null) _colourSource = FindObjectOfType<SimpleImageSubscriber>();
        if (_collector   == null) _collector   = FindObjectOfType<HandEyeCalibrationCollector>();
    }

    bool _initialised;

    void Start()
    {
        BindElements();
        InitXRActions();
    }

    void InitXRActions()
    {
        _xrTabRobot  = new InputAction("XRTabRobot",  InputActionType.Button,
            binding: "<XRController>{LeftHand}/primaryButton");
        _xrHudToggle = new InputAction("XRHudToggle", InputActionType.Button,
            binding: "<XRController>{LeftHand}/thumbstickClicked");

        _xrTabRobot.Enable();
        _xrHudToggle.Enable();
    }

    void OnEnable()
    {
        // Binding is deferred to Start() - OnEnable fires before UIDocument populates its tree.
        if (_collector != null)
        {
            _collector.OnReadyToCapture      += OnReadyToCapture;
            _collector.OnCalibrationComplete += OnCalibrationComplete;
        }
    }

    void BindElements()
    {
        if (_uiDocument == null)
        {
            Debug.LogError("[HUDController] UIDocument not assigned.");
            return;
        }

        var root = _uiDocument.rootVisualElement;
        if (root == null)
        {
            Debug.LogError("[HUDController] rootVisualElement is null - UIDocument may not have a sourceAsset assigned.");
            return;
        }

        // Verify the UXML tree actually instantiated.
        Debug.Log($"[HUDController] visualTreeAsset={(object)_uiDocument.visualTreeAsset ?? (object)"null"}" +
                  $"  root.childCount={root.childCount}");
        if (root.childCount == 0)
            Debug.LogError("[HUDController] rootVisualElement has no children - UXML did not instantiate. " +
                           "Check that sourceAsset is assigned on UIDocument in the scene.");

        _panelRobot = root.Q<VisualElement>("panel-robot");
        _panelCalib = root.Q<VisualElement>("panel-calib");

        var hudRoot = root.Q<VisualElement>("hud-root");
        if (hudRoot != null)
        {
            hudRoot.RegisterCallback<PointerEnterEvent>(_ => IsPointerOver = true);
            hudRoot.RegisterCallback<PointerLeaveEvent>(_ => IsPointerOver = false);
        }

        _tabRobot = root.Q<Button>("tab-robot");
        _tabCalib = root.Q<Button>("tab-calib");

        if (_tabRobot != null) _tabRobot.clicked += () => SetTab(0);
        if (_tabCalib != null) _tabCalib.clicked += () => SetTab(1);

        // Robot panel
        _valEef        = root.Q<Label>("val-eef");
        _valTarget     = root.Q<Label>("val-target");
        _valError      = root.Q<Label>("val-error");
        _valIkMode     = root.Q<Label>("val-ik-mode");
        _valManip      = root.Q<Label>("val-manip");
        _valVel        = root.Q<Label>("val-vel");
        _valState      = root.Q<Label>("val-state");
        _valJointsAbc  = root.Q<Label>("val-joints-abc");
        _valJointsDef  = root.Q<Label>("val-joints-def");
        _valCamFps     = root.Q<Label>("val-cam-fps");
        _valCamRes     = root.Q<Label>("val-cam-res");
        _valCamK       = root.Q<Label>("val-cam-k");

        // Calibration panel
        _valCalibStatus = root.Q<Label>("val-calib-status");
        _btnAuto        = root.Q<Button>("btn-auto");
        _btnManual      = root.Q<Button>("btn-manual");
        _btnConfirm     = root.Q<Button>("btn-confirm");
        _btnFinish      = root.Q<Button>("btn-finish");
        _btnCancel      = root.Q<Button>("btn-cancel");
        _btnLoad        = root.Q<Button>("btn-load");
        _valPairs       = root.Q<Label>("val-pairs");
        _valResidual    = root.Q<Label>("val-residual");
        _valCalibT      = root.Q<Label>("val-calib-t");

        if (_btnAuto    != null) _btnAuto.clicked    += OnAutoClicked;
        if (_btnManual  != null) _btnManual.clicked  += OnManualClicked;
        if (_btnConfirm != null) _btnConfirm.clicked += OnConfirmClicked;
        if (_btnFinish  != null) _btnFinish.clicked  += OnFinishClicked;
        if (_btnCancel  != null) _btnCancel.clicked  += OnCancelClicked;
        if (_btnLoad    != null) _btnLoad.clicked    += OnLoadClicked;

        SetTab(0);

        if (CalibrationResult.Exists())
            UpdateCalibStatus("Calibration on disk", false);
        else
            UpdateCalibStatus("No calibration saved", false);

        if (_tabRobot == null)
            Debug.LogWarning("[HUDController] UXML elements not found - check HUD.uxml loaded correctly (editor-extension-mode=False).");

        _initialised = true;
    }

    void OnDisable()
    {
        if (_collector != null)
        {
            _collector.OnReadyToCapture      -= OnReadyToCapture;
            _collector.OnCalibrationComplete -= OnCalibrationComplete;
        }
        _xrTabRobot?.Dispose();
        _xrHudToggle?.Dispose();
    }

    void Update()
    {
        if (!_initialised) return;

        HandleKeyboard();

        if (!_hudVisible) return;

        switch (_activeTab)
        {
            case 0: UpdateRobotPanel(); break;
            case 1: UpdateCalibPanel(); break;
        }
    }

    void HandleKeyboard()
    {
        var kb = Keyboard.current;
        if (kb == null) return;

        if (kb.f9Key.wasPressedThisFrame)
        {
            _hudVisible = !_hudVisible;
            _uiDocument.rootVisualElement.style.display =
                _hudVisible ? DisplayStyle.Flex : DisplayStyle.None;
        }

        if (kb.f1Key.wasPressedThisFrame) SetTab(0);
        if (kb.f2Key.wasPressedThisFrame) SetTab(1);

        // XR controller buttons (fires when headset inputs are active).
        if (_xrHudToggle != null && _xrHudToggle.WasPressedThisFrame())
        {
            _hudVisible = !_hudVisible;
            _uiDocument.rootVisualElement.style.display =
                _hudVisible ? DisplayStyle.Flex : DisplayStyle.None;
        }
        if (_xrTabRobot != null && _xrTabRobot.WasPressedThisFrame()) SetTab(0);
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
        if (_target == null) return;

        Vector3 eefWorld = _fkSolver != null ? _fkSolver.GetEEFPosition() : Vector3.zero;
        Vector3 base3    = _target.robotBase != null ? _target.robotBase.position : Vector3.zero;
        Vector3 eefLocal = eefWorld - base3;

        SetText(_valEef, $"{eefLocal.x:+0.000;-0.000}  {eefLocal.y:+0.000;-0.000}  {eefLocal.z:+0.000;-0.000} m");

        Vector3 tgtLocal = _target.TargetPosition - base3;
        SetText(_valTarget, $"{tgtLocal.x:+0.000;-0.000}  {tgtLocal.y:+0.000;-0.000}  {tgtLocal.z:+0.000;-0.000} m");
        SetColor(_valTarget, _target.IsControllingTarget ? "#FFA050" : _target.IsInCrosshairRange ? "#FFD020" : "#E6E6F0");

        float errM = (_target.TargetPosition - eefWorld).magnitude;
        SetText(_valError, $"{errM * 1000f:F1} mm   ring {_target.TableNormalisedRadius() * 100f:F0}%");
        SetColor(_valError, errM < 0.005f ? "#3CDD64" : errM < 0.025f ? "#FFD932" : "#FF5046");

        if (_ikSolver != null)
        {
            float  beta    = _ikSolver.LastBlendFactor;
            float  manip   = _ikSolver.LastManipulability;
            string modeStr = beta > 0.5f ? "DLS" : "GD";
            SetText(_valIkMode, $"{modeStr}   beta {beta:F2}");
            SetColor(_valIkMode, ColorLerpHex(new Color(1f, 0.38f, 0.26f), new Color(0.30f, 0.92f, 0.42f), beta));

            SetText(_valManip, manip.ToString("F4"));

            float[] vels    = _ikSolver.LastJointVelocitiesRad;
            float   peak    = 0f;
            if (vels != null)
                for (int i = 0; i < vels.Length; i++)
                    if (Mathf.Abs(vels[i]) > peak) peak = Mathf.Abs(vels[i]);
            float peakPct = peak / Mathf.PI * 100f;
            SetText(_valVel, $"{peakPct:F0}%");
            SetColor(_valVel, ColorLerpHex(new Color(0.30f, 0.92f, 0.42f), new Color(1f, 0.38f, 0.26f), Mathf.Clamp01(peakPct / 100f)));

            var ss = _ikSolver.CurrentSolverState;
            string stateStr;
            string stateCol;
            if (ss == JacobianIKSolver.SolverState.EscapingToHome)
            { stateStr = "HOMING";    stateCol = "#FFAA22"; }
            else if (ss == JacobianIKSolver.SolverState.ResumeDelay)
            { stateStr = "RESUMING";  stateCol = "#FFE633"; }
            else if (_target.IsControllingTarget)
            { stateStr = "CONTROL";   stateCol = "#FFA050"; }
            else if (_target.IsInCrosshairRange)
            { stateStr = "AIMED";     stateCol = "#FFD020"; }
            else
            { stateStr = "IDLE";      stateCol = "#888898"; }
            SetText(_valState, stateStr);
            SetColor(_valState, stateCol);
        }

        if (_publisher != null && _publisher.GetActualJointAnglesInto(_jointBuf))
        {
            SetText(_valJointsAbc, $"{_jointBuf[0]:F1}  {_jointBuf[1]:F1}  {_jointBuf[2]:F1}");
            SetText(_valJointsDef, $"{_jointBuf[3]:F1}  {_jointBuf[4]:F1}  {_jointBuf[5]:F1}");
        }

        if (_renderer != null)
        {
            float fps = _renderer.CameraReceiveFPS;
            SetText(_valCamFps, fps > 0f ? $"{fps:F1} Hz" : "waiting");
            SetColor(_valCamFps, fps > 25f ? "#3CDD64" : fps > 5f ? "#FFD932" : "#FF5046");
            SetText(_valCamRes, $"{_renderer.ColorWidth} x {_renderer.ColorHeight}");
            SetText(_valCamK,   $"fx {_renderer.fx:F1}  fy {_renderer.fy:F1}  cx {_renderer.cx:F1}  cy {_renderer.cy:F1}");
            SetColor(_valCamK, _renderer.IntrinsicsFromDevice ? "#3CDD64" : "#FFD932");
        }
    }

    void UpdateCalibPanel()
    {
        if (_collector == null) return;

        var state = _collector.CurrentState;
        bool running = state != HandEyeCalibrationCollector.State.Idle
                    && state != HandEyeCalibrationCollector.State.Done
                    && state != HandEyeCalibrationCollector.State.Failed;

        bool canFinish = running && _collector.ManualMode
                      && _collector.CapturedCount >= HandEyeSolver.MinPairs;

        bool readyToConfirm = state == HandEyeCalibrationCollector.State.ReadyToCapture;

        SetEnabled(_btnAuto,    !running);
        SetEnabled(_btnManual,  !running);
        SetEnabled(_btnLoad,    !running);
        SetEnabled(_btnCancel,   running);
        SetEnabled(_btnConfirm,  readyToConfirm);
        SetEnabled(_btnFinish,   canFinish);

        SetText(_valPairs, _collector.CapturedCount.ToString());

        if (!float.IsNaN(_collector.ResidualDeg))
        {
            SetText(_valResidual, $"{_collector.ResidualDeg:F2} deg");
            SetColor(_valResidual, _collector.ResidualDeg < 2f ? "#3CDD64"
                                 : _collector.ResidualDeg < 5f ? "#FFD932"
                                 :                               "#FF5046");
        }
        else
        {
            SetText(_valResidual, "--");
        }
    }

    void UpdateCalibStatus(string msg, bool ok)
    {
        SetText(_valCalibStatus, msg);
        SetColor(_valCalibStatus, ok ? "#3CDD64" : "#E6E6F0");
    }

    void OnAutoClicked()
    {
        if (_collector == null) return;
        _collector.SetManualMode(false);
        _collector.StartCalibration();
        UpdateCalibStatus("Auto session started...", false);
    }

    void OnManualClicked()
    {
        if (_collector == null) return;
        _collector.SetManualMode(true);
        _collector.StartCalibration();
        UpdateCalibStatus("Manual session started - move robot, then Confirm", false);
    }

    void OnConfirmClicked()
    {
        _collector?.ConfirmCapture();
    }

    void OnFinishClicked()
    {
        _collector?.FinishManualSession();
    }

    void OnCancelClicked()
    {
        _collector?.CancelCalibration();
        UpdateCalibStatus("Cancelled", false);
    }

    void OnLoadClicked()
    {
        var result = CalibrationResult.Load();
        if (result == null)
        {
            UpdateCalibStatus("No calibration file found", false);
            return;
        }
        _collector?.ApplyCalibrationToScene(result);
        Vector3 t = result.Translation;
        UpdateCalibStatus(
            $"Loaded  residual {result.residualDeg:F2} deg  t ({t.x:+0.00;-0.00}, {t.y:+0.00;-0.00}, {t.z:+0.00;-0.00})",
            result.residualDeg < 5f);
        SetText(_valResidual, $"{result.residualDeg:F2} deg");
        SetText(_valCalibT,   $"{t.x:+0.000;-0.000}  {t.y:+0.000;-0.000}  {t.z:+0.000;-0.000} m");
    }

    void OnReadyToCapture()
    {
        UpdateCalibStatus($"Pose {_collector.CurrentPose + 1} ready - press Confirm", false);
    }

    void OnCalibrationComplete(CalibrationResult result)
    {
        if (result == null)
        {
            UpdateCalibStatus("Calibration failed - not enough pairs", false);
            return;
        }
        Vector3 t = result.Translation;
        UpdateCalibStatus(
            $"Done  residual {result.residualDeg:F2} deg  pairs {result.pairsUsed}",
            result.residualDeg < 5f);
        SetText(_valResidual, $"{result.residualDeg:F2} deg");
        SetText(_valCalibT,   $"{t.x:+0.000;-0.000}  {t.y:+0.000;-0.000}  {t.z:+0.000;-0.000} m");
        SetText(_valPairs,    result.pairsUsed.ToString());
    }

    static void SetText(Label lbl, string text)
    {
        if (lbl != null && lbl.text != text)
            lbl.text = text;
    }

    static void SetColor(Label lbl, string hex)
    {
        if (lbl == null) return;
        lbl.style.color = ParseHex(hex);
    }

    static void SetEnabled(Button btn, bool enabled)
    {
        if (btn == null) return;
        btn.SetEnabled(enabled);
        btn.EnableInClassList("btn-disabled", !enabled);
    }

    static Color ParseHex(string hex)
    {
        if (ColorUtility.TryParseHtmlString(hex, out Color c)) return c;
        return Color.white;
    }

    static string ColorLerpHex(Color a, Color b, float t)
    {
        Color c = Color.Lerp(a, b, t);
        return "#" + ColorUtility.ToHtmlStringRGB(c);
    }

    // Called by WristHUDController to hide/show the pilot HUD while the wrist panel is open.
    public void SetPilotVisible(bool visible)
    {
        _hudVisible = visible;
        if (_uiDocument != null)
            _uiDocument.rootVisualElement.style.display =
                visible ? DisplayStyle.Flex : DisplayStyle.None;
    }
}
