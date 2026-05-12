// Author: Jackson Russell

using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Rendering;
using UnityEngine.InputSystem.XR;

/// Interactive EEF target controller - cylindrical annular work volume with full 6DOF pose control.
///
/// The work volume is a hollow cylinder centred on the robot base:
///   XZ plane : annular ring [tableMinRadius, tableMaxRadius]
///   Y axis   : [tableWorldHeight, tableWorldHeight + taskSpaceHeight]
/// The boundary is visualised at runtime as bottom + top rings with vertical struts.
///
/// EEF POSE: The puck carries a full orientation. TargetPosition is the EEF hover point
///   (approachOffset metres along the puck face normal from the puck anchor). TargetRotation
///   aligns toolApproachAxis into the puck face - like visual servoing against an AR tag.
///
/// ORIENTATION INPUT:
///   XR left thumbstick   - tilt the puck face (pitch + roll, clamped to maxTiltDeg)
///   XR right thumbstick x - spin the puck around its approach axis (tool spin)
///   Keyboard T / G       - pitch tilt forward / back (desktop fallback)
///   Keyboard Q / E       - spin counter-clockwise / clockwise (desktop fallback)
///
/// FPS CROSSHAIR MODE (cursor locked):
///   Short LMB click  - 3-D ray-pick inside the ring; arm arcs to the new position.
///   Long hold + move - drag puck on camera-facing plane (continuous positioning).
///   Scroll wheel     - moves puck up/down within the height volume.
///
/// ARC TRAJECTORY:
///   On a click-target the puck travels around the outer ring boundary rather than
///   cutting straight across. This avoids the wrist-singularity zone at the centre
///   and keeps the path entirely inside the reachable annular volume.
public class EEFTargetController : MonoBehaviour
{
    public static EEFTargetController ActiveInstance { get; private set; }

    [Header("References")]
    public RobotFKSolver fkSolver;
    public Transform     robotBase;

    [Header("Work Surface -- Annular Table")]
    [Tooltip("World-space Y of the table top. Match your scene table height.")]
    public float tableWorldHeight = 6.067f;  // cube pos 5.567 + scale 0.5

    [Tooltip("Inner dead-zone radius from the base in XZ (metres). ~0.10 m for UR3e.")]
    public float tableMinRadius = 0.10f;

    [Tooltip("Outer radius of the work ring (metres). UR3e horizontal reach ~0.40-0.48 m.")]
    public float tableMaxRadius = 0.44f;

    [Tooltip("Height of the work volume above the table surface (metres). Scroll wheel moves puck in this range.")]
    public float taskSpaceHeight = 0.5f;

    [Header("FPS Interaction")]
    [Tooltip("Screen-space pixel radius from screen centre counted as aimed at the puck (orbit mode only; FPS grabs freely).")]
    public float fpsAimPixels = 60f;
    [Tooltip("Metres per scroll unit when adjusting puck height.")]
    public float scrollHeightSpeed = 0.003f;
    [Tooltip("Maximum press duration (seconds) that counts as a click rather than a drag.")]
    public float clickMaxDuration = 0.20f;
    [Tooltip("Maximum mouse movement (px) during a press before it is treated as a drag.")]
    public float clickMaxMovePx   = 12f;

    [Header("EEF Orientation")]
    [Tooltip("Local axis of tool0 that is the gripper approach/face-down direction.\n" +
             "Unity's URDF importer converts ROS Z-approach -> Unity Y, so Vector3.up (0,1,0) is usually correct.\n" +
             "Use Vector3.forward (0,0,1) if the URDF was imported without axis remapping.\n" +
             "Use Vector3.down (0,-1,0) if the arm ends up pointing the wrong way entirely.")]
    public Vector3 toolApproachAxis = Vector3.up;

    [Header("Full Pose Control")]
    [Tooltip("Distance the EEF hovers in front of the puck face along its normal (metres). 0 = EEF at puck surface.")]
    public float approachOffset = 0.05f;

    [Tooltip("Maximum tilt from straight-down (degrees). Prevents impossible joint configurations.")]
    [Range(0f, 85f)]
    public float maxTiltDeg = 60f;

    [Tooltip("Tilt speed via XR left thumbstick or keyboard T/G (degrees per second).")]
    public float tiltSpeed = 60f;

    [Tooltip("Spin speed via XR right thumbstick or keyboard Q/E (degrees per second).")]
    public float spinSpeed = 90f;

    [Header("Arc Trajectory")]
    [Tooltip("Angular sweep speed for the arc phase (degrees/second). Higher = faster arc.")]
    public float arcAngularSpeedDeg = 120f;
    [Tooltip("Fraction of tableMaxRadius the arc path orbits at.  0.9 = near the outer ring, well away from the singularity zone.")]
    [Range(0.5f, 1f)]
    public float arcRadiusFraction  = 0.90f;
    [Tooltip("Angle threshold (deg): if the angular travel is below this the arc phases are skipped and the puck moves direct.")]
    [Range(0f, 90f)]
    public float arcBypassAngleDeg  = 20f;

    [Header("Puck Visuals")]
    [Tooltip("Diameter of the flat puck disc (metres).")]
    public float puckDiameter = 0.12f;
    [Tooltip("Thickness of the puck (metres). Keep small so it sits flat on the table.")]
    public float puckHeight   = 0.015f;
    [Tooltip("Puck colour when idle.")]
    public Color markerColorIdle   = new Color(0.0f, 1.0f, 0.5f, 0.9f);
    [Tooltip("Puck colour when aimed at.")]
    public Color markerColorAimed  = new Color(1.0f, 1.0f, 0.0f, 1.0f);
    [Tooltip("Puck colour while dragging.")]
    public Color markerColorActive = new Color(1.0f, 0.4f, 0.0f, 1.0f);
    [Tooltip("Puck colour while an arc trajectory is playing.")]
    public Color markerColorArc    = new Color(0.3f, 0.6f,  1.0f, 1.0f);

    [Header("Ring Visualisation")]
    [Tooltip("Colour of the inner boundary ring drawn at runtime.")]
    public Color ringInnerColor = new Color(1.0f, 0.35f, 0.1f, 0.7f);
    [Tooltip("Colour of the outer boundary ring drawn at runtime.")]
    public Color ringOuterColor = new Color(0.1f, 0.8f,  1.0f, 0.7f);
    [Tooltip("Width of the ring lines (metres).")]
    public float ringLineWidth = 0.005f;
    [Tooltip("Number of segments per ring line. 64 looks smooth.")]
    public int   ringSegments  = 64;

    // Public state
    public Vector3    TargetPosition      { get; private set; }
    public Quaternion TargetRotation      { get; private set; }
    public bool       IsControllingTarget { get; private set; }
    public bool       IsInCrosshairRange  { get; private set; }
    /// True after the first grab - IK solver stays idle until then.
    public bool       HasBeenGrabbed      { get; private set; }
    /// True while an arc trajectory is actively animating.
    public bool       IsArcActive         => _arcActive;

    // Private
    private GameObject _puck;
    private Renderer   _puckRenderer;
    private Material   _puckMat;
    private Material   _ringMat;
    private Material   _wireMat;
    private Color      _currentPuckColor;

    private Plane       _dragPlane;
    private float       _aimLatchTimer   = 0f;
    private const float AimLatchDuration = 0.25f;
    private const float grabRadius       = 0.06f;

    // Click detection
    private bool    _lmbTracking;
    private float   _lmbDownTime;
    private Vector2 _lmbDownPos;
    private bool    _isDragging;

    // Arc trajectory
    private bool    _arcActive;
    private float   _arcR0, _arcTheta0, _arcY0;   // cylindrical source
    private float   _arcRa;                        // orbit radius during sweep
    private float   _arcR1, _arcTheta1, _arcY1;   // cylindrical destination
    private float   _arcT;                         // 0..1 progress
    private float   _arcDuration;                  // seconds
    private Vector3 _finalTarget;                  // exact destination (snapped at end)

    private Camera _cam;

    // XR controller input
    private InputAction  _xrTriggerAction;
    private bool         _xrTrigTracking;
    private float        _xrTrigDownTime;
    private bool         _xrTrigDragging;
    private LineRenderer _xrPointerLine;
    private Material     _xrPointerMat;
    private bool         _xrAimingAtPuck;          // true when ray is close to puck
    private const float  XRHoverRadius = 0.12f;    // metres — ray-tip within this = hover

    // Full pose control.
    // _puckAnchor is the physical world-space puck position.
    // TargetPosition is derived: _puckAnchor + face normal * approachOffset.
    // TargetRotation is derived: toolApproachAxis aligned into the puck face.
    private Vector3    _puckAnchor   = Vector3.zero;
    private float      _puckPitch    = 0f;   // tilt forward/back (degrees)
    private float      _puckRoll     = 0f;   // tilt left/right (degrees)
    private float      _puckSpin     = 0f;   // spin around approach axis (degrees)
    private Quaternion _puckRotation = Quaternion.identity;

    // XR input actions for puck orientation control.
    private InputAction  _xrLeftStickAction;
    private InputAction  _xrRightStickAction;

    // LineRenderer from puck face center to EEF hover point, showing approach direction.
    private LineRenderer _approachArrow;
    private Material     _approachArrowMat;

    void Awake() { ActiveInstance = this; }
    void OnDestroy()
    {
        if (_wireMat) Destroy(_wireMat);
        if (_ringMat) Destroy(_ringMat);
        if (_approachArrowMat) Destroy(_approachArrowMat);
        _xrTriggerAction?.Dispose();
        _xrLeftStickAction?.Dispose();
        _xrRightStickAction?.Dispose();
    }

    void Start()
    {
        _cam = Camera.main;

        // XR trigger+grip action - either right-hand trigger or grip activates puck control.
        _xrTriggerAction = new InputAction("XRSelect", InputActionType.Button);
        _xrTriggerAction.AddBinding("<XRController>{RightHand}/triggerButton");
        _xrTriggerAction.AddBinding("<XRController>{RightHand}/gripButton");
        _xrTriggerAction.Enable();

        // Left thumbstick controls puck tilt, right thumbstick x-axis controls spin.
        _xrLeftStickAction = new InputAction("XRLeftStick", InputActionType.Value,
            binding: "<XRController>{LeftHand}/thumbstick");
        _xrLeftStickAction.Enable();

        _xrRightStickAction = new InputAction("XRRightStick", InputActionType.Value,
            binding: "<XRController>{RightHand}/thumbstick");
        _xrRightStickAction.Enable();

        // Seed target at the current FK EEF position projected onto the table surface.
        // Zero initial IK error keeps the arm still. Solver also gated by HasBeenGrabbed.
        Vector3 eefWorld = fkSolver != null
            ? fkSolver.GetEEFPosition()
            : (robotBase != null ? robotBase.position : Vector3.zero);
        ClampAndSetTarget(new Vector3(eefWorld.x, tableWorldHeight, eefWorld.z));
        RefreshPose();

        // Flat puck disc (cylinder squashed to puckHeight)
        _puck = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        _puck.name = "EEF_Puck";
        _puck.transform.localScale = new Vector3(puckDiameter, puckHeight * 0.5f, puckDiameter);
        Destroy(_puck.GetComponent<Collider>());

        _puckRenderer = _puck.GetComponent<Renderer>();
        // Use .material (not .sharedMaterial) to get a unique instanced copy.
        // Unity auto-assigns the correct shader for the active RP to any primitive
        _puckMat = _puckRenderer.material;
        _currentPuckColor = markerColorIdle;
        SetPuckColor(_currentPuckColor);
        // Lift by half height so the flat base sits exactly on the table surface.
        _puck.transform.position = _puckAnchor + Vector3.up * (puckHeight * 0.5f);
        _puck.transform.rotation = Quaternion.identity;

        // Arrow LineRenderer from puck face center to the EEF hover point.
        var arrowGo = new GameObject("PuckApproachArrow");
        arrowGo.transform.SetParent(_puck.transform, false);
        _approachArrow              = arrowGo.AddComponent<LineRenderer>();
        _approachArrow.positionCount = 2;
        _approachArrow.startWidth   = 0.006f;
        _approachArrow.endWidth     = 0.002f;
        _approachArrow.useWorldSpace = true;
        _approachArrow.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        _approachArrowMat = new Material(_puckMat);
        ApplyColor(_approachArrowMat, new Color(0.8f, 0.9f, 1f, 0.85f));
        _approachArrow.material = _approachArrowMat;

        // Runtime volume cage (LineRenderer)
        _ringMat = new Material(_puckMat);
        _wireMat = null;
        float yBot = tableWorldHeight + 0.005f;
        float yTop = tableWorldHeight + taskSpaceHeight;
        BuildRingLineRenderer("Ring_Bot_Inner", tableMinRadius, yBot, ringInnerColor);
        BuildRingLineRenderer("Ring_Bot_Outer", tableMaxRadius, yBot, ringOuterColor);
        BuildRingLineRenderer("Ring_Top_Inner", tableMinRadius, yTop, ringInnerColor);
        BuildRingLineRenderer("Ring_Top_Outer", tableMaxRadius, yTop, ringOuterColor);
        // Four vertical struts on the outer ring at NESW.
        float bx2 = robotBase != null ? robotBase.position.x : 0f;
        float bz2 = robotBase != null ? robotBase.position.z : 0f;
        foreach (float angle in new[] { 0f, 90f, 180f, 270f })
        {
            float rad = angle * Mathf.Deg2Rad;
            Vector3 bottom = new Vector3(bx2 + Mathf.Sin(rad) * tableMaxRadius, yBot, bz2 + Mathf.Cos(rad) * tableMaxRadius);
            Vector3 top    = new Vector3(bx2 + Mathf.Sin(rad) * tableMaxRadius, yTop, bz2 + Mathf.Cos(rad) * tableMaxRadius);
            BuildStrut($"Strut_{angle:0}", bottom, top, ringOuterColor);
        }

        // Print exact spawn positions to verify scene matches config params.
        float rbx = robotBase != null ? robotBase.position.x : 0f;
        float rbz = robotBase != null ? robotBase.position.z : 0f;
        Debug.Log($"[EEFTarget] robotBase = {(robotBase != null ? robotBase.position.ToString() : "NULL")}");
        Debug.Log($"[EEFTarget] Volume bottom Y={tableWorldHeight:F3}  top Y={(tableWorldHeight+taskSpaceHeight):F3}");
        Debug.Log($"[EEFTarget] Puck spawned at {_puck.transform.position}");
        Debug.Log($"[EEFTarget] Ring centre ({rbx:F3}, *, {rbz:F3})  r_in={tableMinRadius}  r_out={tableMaxRadius}");
    }

    // Createhorizontal circle LineRenderer at height y around robotBase.
    void BuildRingLineRenderer(string objName, float radius, float y, Color color)
    {
        var go = new GameObject(objName);
        go.transform.SetParent(transform);
        var lr = go.AddComponent<LineRenderer>();

        var mat = new Material(_ringMat);
        ApplyColor(mat, color);
        lr.material      = mat;
        lr.startColor    = color;
        lr.endColor      = color;
        lr.startWidth    = ringLineWidth;
        lr.endWidth      = ringLineWidth;
        lr.loop          = true;
        lr.useWorldSpace = true;
        lr.positionCount = ringSegments;
        lr.numCapVertices = 2;
        lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        lr.receiveShadows = false;

        float bx = robotBase != null ? robotBase.position.x : 0f;
        float bz = robotBase != null ? robotBase.position.z : 0f;
        for (int i = 0; i < ringSegments; i++)
        {
            float a = i * Mathf.PI * 2f / ringSegments;
            lr.SetPosition(i, new Vector3(bx + Mathf.Sin(a) * radius, y, bz + Mathf.Cos(a) * radius));
        }
    }

    // Create 2-point LineRenderer strut between two world positions.
    void BuildStrut(string objName, Vector3 bottom, Vector3 top, Color color)
    {
        var go = new GameObject(objName);
        go.transform.SetParent(transform);
        var lr = go.AddComponent<LineRenderer>();

        var mat = new Material(_ringMat);
        ApplyColor(mat, color);
        lr.material      = mat;
        lr.startColor    = color;
        lr.endColor      = color;
        lr.startWidth    = ringLineWidth;
        lr.endWidth      = ringLineWidth;
        lr.loop          = false;
        lr.useWorldSpace = true;
        lr.positionCount = 2;
        lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        lr.receiveShadows = false;
        lr.SetPosition(0, bottom);
        lr.SetPosition(1, top);
    }

    void Update()
    {
        if (_cam == null) { _cam = Camera.main; return; }

        // Orientation input: XR left stick = tilt (pitch/roll), right stick x = spin.
        // Desktop fallback: T/G = pitch tilt forward/back, Q/E = spin.
        if (XRRigSetup.Instance != null && XRRigSetup.Instance.IsXRActive)
        {
            Vector2 leftStick  = _xrLeftStickAction  != null
                ? _xrLeftStickAction.ReadValue<Vector2>()  : Vector2.zero;
            Vector2 rightStick = _xrRightStickAction != null
                ? _xrRightStickAction.ReadValue<Vector2>() : Vector2.zero;

            _puckPitch = Mathf.Clamp(_puckPitch - leftStick.y * tiltSpeed * Time.deltaTime, -maxTiltDeg, maxTiltDeg);
            _puckRoll  = Mathf.Clamp(_puckRoll  + leftStick.x * tiltSpeed * Time.deltaTime, -maxTiltDeg, maxTiltDeg);
            _puckSpin += rightStick.x * spinSpeed * Time.deltaTime;
        }
        else
        {
            var kb = Keyboard.current;
            if (kb != null)
            {
                if (kb.tKey.isPressed) _puckPitch = Mathf.Clamp(_puckPitch - tiltSpeed * Time.deltaTime, -maxTiltDeg, maxTiltDeg);
                if (kb.gKey.isPressed) _puckPitch = Mathf.Clamp(_puckPitch + tiltSpeed * Time.deltaTime, -maxTiltDeg, maxTiltDeg);
                if (kb.qKey.isPressed) _puckSpin -= spinSpeed * Time.deltaTime;
                if (kb.eKey.isPressed) _puckSpin += spinSpeed * Time.deltaTime;
            }
        }

        // Clamp total tilt so the diagonal does not exceed maxTiltDeg.
        float totalTilt = Mathf.Sqrt(_puckPitch * _puckPitch + _puckRoll * _puckRoll);
        if (totalTilt > maxTiltDeg)
        {
            float scale = maxTiltDeg / totalTilt;
            _puckPitch *= scale;
            _puckRoll  *= scale;
        }

        bool cursorLocked = Cursor.lockState == CursorLockMode.Locked;

        bool rawInRange = cursorLocked || IsAimedAt(false, _puckAnchor);
        if (rawInRange) _aimLatchTimer = AimLatchDuration;
        else            _aimLatchTimer -= Time.deltaTime;
        IsInCrosshairRange = rawInRange || _aimLatchTimer > 0f;

        // XR controller input (right hand trigger)
        if (XRRigSetup.Instance != null && XRRigSetup.Instance.IsXRActive)
        {
            UpdateXRPointer();
            HandleXRTrigger();
        }
        else
        {
            // Desktop mouse input
            var  mouse   = Mouse.current;
            bool lmbDown = mouse.leftButton.wasPressedThisFrame;
            bool lmbHeld = mouse.leftButton.isPressed;
            bool lmbUp   = mouse.leftButton.wasReleasedThisFrame;

            // FPS drag-release: if already dragging and the user presses LMB
            // again (cursor-locked toggle) stop drag immediately and don't start a new one.
            if (cursorLocked && IsControllingTarget && lmbDown)
            {
                IsControllingTarget = false;
                _lmbTracking        = false;
                _isDragging         = false;
            }
            else
            {
                // Click / drag discrimination
                // A press shorter than clickMaxDuration with small mouse movement = click.
                // A longer press or significant movement = drag (continuous positioning).
                if (lmbDown)
                {
                    _lmbTracking = true;
                    _lmbDownTime = Time.time;
                    _lmbDownPos  = mouse.position.ReadValue();
                    _isDragging  = false;
                }

                // Upgrade to drag mid-hold once threshold is crossed.
                if (_lmbTracking && lmbHeld && !_isDragging && IsInCrosshairRange && !IsControllingTarget)
                {
                    float moved = (mouse.position.ReadValue() - _lmbDownPos).magnitude;
                    if (Time.time - _lmbDownTime > clickMaxDuration || moved > clickMaxMovePx)
                    {
                        _isDragging         = true;
                        IsControllingTarget = true;
                        HasBeenGrabbed      = true;
                        _arcActive          = false;   // interrupt arc when grabbed
                    }
                }

                if (lmbUp && _lmbTracking)
                {
                    if (_isDragging)
                    {
                        // End of drag.
                        IsControllingTarget = false;
                    }
                    else if (IsInCrosshairRange)
                    {
                        // Short click -> 3-D ray-pick + arc trajectory.
                        if (TryPickTargetByRay(out Vector3 picked))
                            StartArcTo(picked);
                    }
                    _lmbTracking = false;
                    _isDragging  = false;
                }
            }

            // Continuous drag update
            if (IsControllingTarget)
            {
                // Re-anchor every frame so the puck tracks the crosshair exactly.
                _dragPlane = new Plane(-_cam.transform.forward, _puckAnchor);
                DragOnTable(mouse);
            }
        }

        // Advance arc (runs only when not manually controlled)
        if (_arcActive && !IsControllingTarget)
            AdvanceArc();

        // Puck colour — XR hover overrides the normal aimed colour
        bool aimed = IsInCrosshairRange || _xrAimingAtPuck;
        Color targetColor = IsControllingTarget ? markerColorActive
                          : _arcActive          ? markerColorArc
                          : aimed               ? markerColorAimed
                          :                       markerColorIdle;
        _currentPuckColor = Color.Lerp(_currentPuckColor, targetColor, Time.deltaTime * 12f);
        SetPuckColor(_currentPuckColor);

        // Scale puck up slightly when XR ray is hovering to make it easier to see.
        float targetScale = (IsControllingTarget || _xrAimingAtPuck) ? 1.35f : 1f;
        float curScale    = _puck.transform.localScale.x / puckDiameter;
        float newScale    = Mathf.Lerp(curScale, targetScale, Time.deltaTime * 10f);
        _puck.transform.localScale = new Vector3(
            puckDiameter * newScale, puckHeight * 0.5f, puckDiameter * newScale);

        // Derive TargetPosition and TargetRotation, then update puck mesh pose.
        RefreshPose();
        _puck.transform.rotation = _puckRotation;
        _puck.transform.position = _puckAnchor + _puckRotation * Vector3.up * (puckHeight * 0.5f);

        // Point approach arrow from puck face center toward the EEF hover target.
        if (_approachArrow != null)
        {
            _approachArrow.SetPosition(0, _puckAnchor);
            _approachArrow.SetPosition(1, TargetPosition);
            ApplyColor(_approachArrowMat, _currentPuckColor * 1.2f);
        }
    }

    // XR controller methods

    // Draw pointer line from the right controller; highlight when aimed at puck.
    void UpdateXRPointer()
    {
        var ctrl = XRRigSetup.Instance.RightController;
        if (ctrl == null) return;

        // Lazy-create pointer LineRenderer.
        if (_xrPointerLine == null)
        {
            var go = new GameObject("XR_PointerLine");
            go.transform.SetParent(ctrl, false);
            _xrPointerLine             = go.AddComponent<LineRenderer>();
            _xrPointerLine.positionCount = 2;
            _xrPointerLine.startWidth  = 0.005f;
            _xrPointerLine.endWidth    = 0.002f;
            _xrPointerLine.useWorldSpace = true;
            _xrPointerLine.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            if (_puckMat != null)
            {
                _xrPointerMat = new Material(_puckMat);
                _xrPointerLine.material = _xrPointerMat;
            }
        }

        Ray ray = new Ray(ctrl.position, ctrl.forward);
        TryPickByControllerRay(ray, out Vector3 hit);

        // Check how close the ray tip is to the puck.
        float distToPuck = Vector3.Distance(hit, _puckAnchor);
        _xrAimingAtPuck  = distToPuck < XRHoverRadius;

        // Snap the tip to the puck when hovering so it's obvious.
        Vector3 tip = _xrAimingAtPuck ? _puckAnchor : hit;

        // Line colour: green = hovering over puck, cyan = free aim.
        Color lineCol = _xrAimingAtPuck
            ? new Color(0.2f, 1f, 0.3f, 1f)
            : new Color(0.2f, 0.8f, 1f, 0.6f);
        if (_xrPointerMat != null) ApplyColor(_xrPointerMat, lineCol);
        _xrPointerLine.startWidth = _xrAimingAtPuck ? 0.007f : 0.004f;

        _xrPointerLine.SetPosition(0, ctrl.position);
        _xrPointerLine.SetPosition(1, tip);
    }

    // Click / drag with the right-hand trigger.
    void HandleXRTrigger()
    {
        var ctrl = XRRigSetup.Instance.RightController;
        if (ctrl == null || _xrTriggerAction == null) return;

        bool trigDown = _xrTriggerAction.WasPressedThisFrame();
        bool trigHeld = _xrTriggerAction.IsPressed();
        bool trigUp   = _xrTriggerAction.WasReleasedThisFrame();

        Ray ray = new Ray(ctrl.position, ctrl.forward);

        if (trigDown)
        {
            _xrTrigTracking = true;
            _xrTrigDownTime = Time.time;
            _xrTrigDragging = false;
        }

        // Upgrade to drag after hold threshold.
        if (_xrTrigTracking && trigHeld && !_xrTrigDragging)
        {
            if (Time.time - _xrTrigDownTime > clickMaxDuration)
            {
                _xrTrigDragging     = true;
                IsControllingTarget = true;
                HasBeenGrabbed      = true;
                _arcActive          = false;
            }
        }

        if (trigUp && _xrTrigTracking)
        {
            if (_xrTrigDragging)
            {
                IsControllingTarget = false;
            }
            else
            {
                // Short press → arc to target.
                if (TryPickByControllerRay(ray, out Vector3 picked))
                    StartArcTo(picked);
            }
            _xrTrigTracking = false;
            _xrTrigDragging = false;
        }

        // Continuous drag: place puck at ray-vs-table-plane intersection every frame.
        if (IsControllingTarget && _xrTrigDragging)
        {
            var tablePlane = new Plane(Vector3.up, new Vector3(0f, _puckAnchor.y, 0f));
            if (tablePlane.Raycast(ray, out float enter))
                ClampAndSetTarget(ray.GetPoint(enter));
        }
    }

    // Cast a ray from the controller and intersect with the work volume.
    bool TryPickByControllerRay(Ray ray, out Vector3 result)
    {
        result = _puckAnchor;

        var hPlane = new Plane(Vector3.up, new Vector3(0f, _puckAnchor.y, 0f));
        if (hPlane.Raycast(ray, out float ht))
        {
            result = ClampToVolume(ray.GetPoint(ht));
            return true;
        }

        // Fallback: cylinder side-wall.
        float bx = robotBase != null ? robotBase.position.x : 0f;
        float bz = robotBase != null ? robotBase.position.z : 0f;
        if (!IntersectVerticalCylinder(ray, bx, bz, tableMaxRadius, out float tOA, out float tOB))
            return false;
        float tEnter = Mathf.Max(0f, Mathf.Min(tOA, tOB));
        result = ClampToVolume(ray.GetPoint(tEnter));
        return true;
    }

    // Drag: cast the aim ray (screen centre) against the camera-facing plane.
    // XZ position comes from the raycast hit; Y is unchanged by mouse movement.
    // Scroll wheel moves the puck up/down within the work volume.
    void DragOnTable(Mouse m)
    {
        if (m == null || _cam == null) return;

        Ray ray = new Ray(_cam.transform.position, _cam.transform.forward);
        if (_dragPlane.Raycast(ray, out float enter))
        {
            Vector3 hit = ray.GetPoint(enter);
            ClampAndSetTarget(new Vector3(hit.x, _puckAnchor.y, hit.z));
        }

        float scroll = m.scroll.ReadValue().y;
        if (Mathf.Abs(scroll) > 0.01f)
        {
            float newY = Mathf.Clamp(_puckAnchor.y + scroll * scrollHeightSpeed,
                                     tableWorldHeight,
                                     tableWorldHeight + taskSpaceHeight);
            _puckAnchor = new Vector3(_puckAnchor.x, newY, _puckAnchor.z);
        }
    }

    /// Sets position and rotation directly from an external source (e.g. IMUSubscriber).
    /// Position is clamped to the configured work volume.
    public void SetTarget(Vector3 position, Quaternion rotation)
    {
        ClampAndSetTarget(position);
        // Decompose rotation to euler so the incremental input keeps working after an external set.
        Vector3 e  = rotation.eulerAngles;
        _puckPitch = e.x > 180f ? e.x - 360f : e.x;
        _puckSpin  = e.y;
        _puckRoll  = e.z > 180f ? e.z - 360f : e.z;
        HasBeenGrabbed = true;
    }

    // Clamp XZ to annular ring and Y to work volume.
    void ClampAndSetTarget(Vector3 world) => _puckAnchor = ClampToVolume(world);

    // Recomputes TargetPosition and TargetRotation from the puck anchor and current orientation.
    // TargetPosition is the EEF hover point, approachOffset metres along the puck face normal.
    // TargetRotation aligns toolApproachAxis to point into the puck face.
    void RefreshPose()
    {
        _puckRotation  = Quaternion.Euler(_puckPitch, _puckSpin, _puckRoll);
        Vector3 normal = _puckRotation * Vector3.up;
        TargetPosition = _puckAnchor + normal * approachOffset;
        TargetRotation = Quaternion.FromToRotation(toolApproachAxis.normalized, -normal);
    }

    Vector3 ClampToVolume(Vector3 world)
    {
        float bx = robotBase != null ? robotBase.position.x : 0f;
        float bz = robotBase != null ? robotBase.position.z : 0f;

        world.y = Mathf.Clamp(world.y, tableWorldHeight, tableWorldHeight + taskSpaceHeight);

        Vector2 offset = new Vector2(world.x - bx, world.z - bz);
        float r = offset.magnitude;
        if      (r < 1e-4f)          offset = new Vector2(0f, tableMinRadius);
        else if (r < tableMinRadius) offset = offset.normalized * tableMinRadius;
        else if (r > tableMaxRadius) offset = offset.normalized * tableMaxRadius;

        return new Vector3(bx + offset.x, world.y, bz + offset.y);
    }

    // 3-D Click: ray -> work-volume intersection

    /// Casts a ray from the camera and returns the XZ position where the crosshair
    /// intersects the horizontal plane at the current puck height, clamped to the
    /// annular ring.  Falls back to the cylinder side-wall for near-horizontal rays.
    bool TryPickTargetByRay(out Vector3 result)
    {
        result = _puckAnchor;
        if (_cam == null) return false;

        Ray ray = (Cursor.lockState == CursorLockMode.Locked)
                ? new Ray(_cam.transform.position, _cam.transform.forward)
                : _cam.ScreenPointToRay(Mouse.current.position.ReadValue());

        float bx   = robotBase != null ? robotBase.position.x : 0f;
        float bz   = robotBase != null ? robotBase.position.z : 0f;

        // Primary: intersect with the horizontal plane at the current puck height.
        // This correctly maps the crosshair to an arbitrary XZ position inside the ring,
        // regardless of whether the camera is inside or outside the outer boundary.
        var hPlane = new Plane(Vector3.up, new Vector3(0f, _puckAnchor.y, 0f));
        if (hPlane.Raycast(ray, out float ht))
        {
            result = ClampToVolume(ray.GetPoint(ht));
            return true;
        }

        // Fallback for near-horizontal rays: cylinder side-wall intersection.
        float tEnter = 0f;
        float tExit  = float.MaxValue;

        if (!IntersectVerticalCylinder(ray, bx, bz, tableMaxRadius, out float tOA, out float tOB))
            return false;
        tEnter = Mathf.Max(tEnter, Mathf.Min(tOA, tOB));
        tExit  = Mathf.Min(tExit,  Mathf.Max(tOA, tOB));
        if (tEnter > tExit) return false;

        if (IntersectVerticalCylinder(ray, bx, bz, tableMinRadius, out float tIA, out float tIB))
        {
            float tIEnter = Mathf.Min(tIA, tIB);
            float tIExit  = Mathf.Max(tIA, tIB);
            if (tIEnter <= tEnter && tIExit >= tExit) return false;
            if (tIEnter <= tEnter) tEnter = tIExit;
        }

        if (tEnter > tExit || tExit < 0f) return false;

        result = ClampToVolume(ray.GetPoint(Mathf.Max(tEnter, 0f)));
        return true;
    }

    /// Intersects a ray with an infinite vertical cylinder at (cx, *, cz).
    /// Returns false if the ray is parallel to the axis or entirely outside.
    static bool IntersectVerticalCylinder(Ray ray,
                                          float cx, float cz, float radius,
                                          out float t0, out float t1)
    {
        t0 = t1 = 0f;
        float ox = ray.origin.x - cx,  oz = ray.origin.z - cz;
        float dx = ray.direction.x,    dz = ray.direction.z;
        float a  = dx * dx + dz * dz;
        if (a < 1e-10f) return false;          // ray parallel to cylinder axis
        float b    = 2f * (ox * dx + oz * dz);
        float c    = ox * ox + oz * oz - radius * radius;
        float disc = b * b - 4f * a * c;
        if (disc < 0f) return false;
        float sq = Mathf.Sqrt(disc);
        t0 = (-b - sq) / (2f * a);
        t1 = (-b + sq) / (2f * a);
        return true;
    }

    // Arc trajectory: cylindrical arc around the outer ring boundary

    /// Start an arc trajectory from the current TargetPosition to <dest>.
    /// The path passes through the outer ring boundary arc so the arm never
    /// cuts across the singularity zone near the base centre.
    void StartArcTo(Vector3 dest)
    {
        dest         = ClampToVolume(dest);
        _finalTarget = dest;
        HasBeenGrabbed = true;

        float bx = robotBase != null ? robotBase.position.x : 0f;
        float bz = robotBase != null ? robotBase.position.z : 0f;

        // Decompose source and destination into cylindrical coordinates.
        Vector2 src2 = new Vector2(_puckAnchor.x - bx, _puckAnchor.z - bz);
        _arcR0     = Mathf.Clamp(src2.magnitude, tableMinRadius, tableMaxRadius);
        _arcTheta0 = Mathf.Atan2(src2.x, src2.y);   // angle from +Z in XZ plane
        _arcY0     = _puckAnchor.y;

        Vector2 dst2 = new Vector2(dest.x - bx, dest.z - bz);
        _arcR1     = Mathf.Clamp(dst2.magnitude, tableMinRadius, tableMaxRadius);
        _arcTheta1 = Mathf.Atan2(dst2.x, dst2.y);
        _arcY1     = dest.y;

        // Orbit radius hugs the outer boundary (avoids singularity at centre).
        _arcRa = Mathf.Clamp(tableMaxRadius * arcRadiusFraction, tableMinRadius, tableMaxRadius);

        // Choose the shortest angular direction.
        float dThetaDeg = Mathf.DeltaAngle(_arcTheta0 * Mathf.Rad2Deg, _arcTheta1 * Mathf.Rad2Deg);
        float dTheta    = dThetaDeg * Mathf.Deg2Rad;       // signed, shortest path
        _arcTheta1      = _arcTheta0 + dTheta;             // consistent with source angle

        float linearSpeed = _arcRa * arcAngularSpeedDeg * Mathf.Deg2Rad;  // m/s

        if (Mathf.Abs(dThetaDeg) < arcBypassAngleDeg)
        {
            // Small angle -> interpolate directly through the volume.
            _arcRa = (_arcR0 + _arcR1) * 0.5f;      // dummy "arc radius"; phases still work
            _arcDuration = Vector3.Distance(_puckAnchor, dest) / Mathf.Max(linearSpeed, 0.01f);
        }
        else
        {
            // Full three-phase arc (radial-in -> sweep angle -> radial-out).
            float arcLength    = _arcRa * Mathf.Abs(dTheta);
            float radialLength = Mathf.Abs(_arcR0 - _arcRa) + Mathf.Abs(_arcRa - _arcR1);
            _arcDuration = (arcLength + radialLength) / Mathf.Max(linearSpeed, 0.01f);
        }
        _arcDuration = Mathf.Clamp(_arcDuration, 0.15f, 6f);

        _arcT      = 0f;
        _arcActive = true;
    }

    /// Advance TargetPosition along the arc each frame.
    void AdvanceArc()
    {
        _arcT += Time.deltaTime / Mathf.Max(_arcDuration, 0.01f);
        _arcT  = Mathf.Clamp01(_arcT);

        float r, theta, y;

        // Three-phase path inside the annular cylinder:
        //   [0.0 - 0.2]  radial move: r0 -> r_arc  (approach the orbit ring)
        //   [0.2 - 0.8]  angle sweep: theta0 -> theta1 at r_arc, height interpolates
        //   [0.8 - 1.0]  radial move: r_arc -> r1   (move to final radius)
        if (_arcT < 0.2f)
        {
            float p = _arcT / 0.2f;
            r     = Mathf.Lerp(_arcR0, _arcRa, p);
            theta = _arcTheta0;
            y     = _arcY0;
        }
        else if (_arcT < 0.8f)
        {
            float p = (_arcT - 0.2f) / 0.6f;
            r     = _arcRa;
            theta = Mathf.Lerp(_arcTheta0, _arcTheta1, p);
            y     = Mathf.Lerp(_arcY0, _arcY1, p);
        }
        else
        {
            float p = (_arcT - 0.8f) / 0.2f;
            r     = Mathf.Lerp(_arcRa, _arcR1, p);
            theta = _arcTheta1;
            y     = _arcY1;
        }

        if (_arcT >= 1f)
        {
            _arcActive  = false;
            _puckAnchor = _finalTarget;
            return;
        }

        float bx = robotBase != null ? robotBase.position.x : 0f;
        float bz = robotBase != null ? robotBase.position.z : 0f;
        _puckAnchor = new Vector3(
            bx + r * Mathf.Sin(theta),
            y,
            bz + r * Mathf.Cos(theta));
    }

    // 0-1 normalised radial position within the ring. Used by CrosshairHUD.
    public float TableNormalisedRadius()
    {
        if (robotBase == null) return 0f;
        Vector2 o    = new Vector2(_puckAnchor.x - robotBase.position.x,
                                   _puckAnchor.z - robotBase.position.z);
        float span = tableMaxRadius - tableMinRadius;
        return span > 1e-4f ? Mathf.Clamp01((o.magnitude - tableMinRadius) / span) : 0f;
    }

    // Aim check: screen pixels (FPS) or world distance (orbit).
    bool IsAimedAt(bool cursorLocked, Vector3 worldPoint)
    {
        if (cursorLocked)
        {
            Vector3 s = _cam.WorldToScreenPoint(worldPoint);
            if (s.z <= 0f) return false;
            float dx = s.x - Screen.width  * 0.5f;
            float dy = s.y - Screen.height * 0.5f;
            return dx * dx + dy * dy < fpsAimPixels * fpsAimPixels;
        }
        Ray ray = _cam.ScreenPointToRay(Mouse.current.position.ReadValue());
        return Vector3.Cross(ray.direction, worldPoint - ray.origin).magnitude < grabRadius;
    }

    // Sets puck colour: _BaseColor (URP), legacy _Color, and emissive glow.
    void SetPuckColor(Color c)
    {
        if (_puckMat == null) return;
        ApplyColor(_puckMat, c);
    }

    static void ApplyColor(Material mat, Color c)
    {
        mat.SetColor("_BaseColor",     c);
        mat.color = c;
        mat.EnableKeyword("_EMISSION");
        mat.SetColor("_EmissionColor", c * 1.5f);
    }

    void OnDrawGizmosSelected()
    {
        if (robotBase == null) return;
        Vector3 c = new Vector3(robotBase.position.x, tableWorldHeight, robotBase.position.z);
        DrawCircleGizmo(c, tableMinRadius,                   new Color(1f, 0.3f, 0.1f, 0.8f), 64);
        DrawCircleGizmo(c, tableMaxRadius,                   new Color(0f, 0.8f, 1f,   0.8f), 64);
        // Arc orbit ring shown in blue (the path the arm sweeps around).
        DrawCircleGizmo(c, tableMaxRadius * arcRadiusFraction, new Color(0.3f, 0.6f, 1f, 0.5f), 48);
        for (int i = 0; i < 8; i++)
        {
            float a  = i * Mathf.PI * 2f / 8f;
            Gizmos.color = new Color(0f, 0.8f, 1f, 0.2f);
            Gizmos.DrawLine(c + new Vector3(Mathf.Sin(a) * tableMinRadius, 0f, Mathf.Cos(a) * tableMinRadius),
                            c + new Vector3(Mathf.Sin(a) * tableMaxRadius, 0f, Mathf.Cos(a) * tableMaxRadius));
        }
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(robotBase.position, c);
    }

    static void DrawCircleGizmo(Vector3 centre, float radius, Color color, int segments)
    {
        Gizmos.color = color;
        float step = Mathf.PI * 2f / segments;
        Vector3 prev = centre + new Vector3(Mathf.Sin(0) * radius, 0f, Mathf.Cos(0) * radius);
        for (int i = 1; i <= segments; i++)
        {
            float   a    = i * step;
            Vector3 next = centre + new Vector3(Mathf.Sin(a) * radius, 0f, Mathf.Cos(a) * radius);
            Gizmos.DrawLine(prev, next);
            prev = next;
        }
    }
}

