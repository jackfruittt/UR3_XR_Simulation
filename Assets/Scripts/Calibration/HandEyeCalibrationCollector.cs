// Author: Jackson Russell

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// Semi-autonomous hand-eye calibration collector for the UR3e (eye-in-hand).
///
/// The operator selects "Calibrate" from the settings panel. The collector
/// commands the robot through a pre-defined set of waypoints one at a time.
/// At each waypoint the user confirms the capture (or it triggers automatically
/// once the robot has settled and a calibration tag is stably detected).
/// After all waypoints are processed the AX=XB solver is invoked and the
/// result is written to disk via CalibrationResult.
///
/// Requires:
///   publisher      - UR3SourceDestinationPublisher (joint commands + FK)
///   fkSolver       - RobotFKSolver (tool0 world pose)
///   detection      - Detection (AprilTag pose in camera frame)
///   targetTagId    - ID of the AprilTag used as the calibration target
///
/// Reference:
///   Tsai, R.Y. and Lenz, R.K. (1989). A new technique for fully autonomous
///   and efficient 3D robotics hand/eye calibration
public class HandEyeCalibrationCollector : MonoBehaviour
{
    // Inspector fields

    [Header("Dependencies")]
    [SerializeField] UR3SourceDestinationPublisher _publisher;
    [SerializeField] RobotFKSolver                 _fkSolver;
    [SerializeField] Detection                     _detection;

    [Header("Calibration Settings")]
    [Tooltip("ID of the AprilTag used as the fixed calibration target.")]
    [SerializeField] int _targetTagId = 0;

    [Tooltip("Seconds the robot must remain within the settle threshold before capture is armed.")]
    [SerializeField] float _settleTime = 0.8f;

    [Tooltip("Joint angle error (degrees) below which the robot is considered settled.")]
    [SerializeField] float _settleThresholdDeg = 0.5f;

    [Tooltip("Consecutive detection frames required before a capture is considered valid.")]
    [SerializeField] int _stableTagFrames = 10;

    [Tooltip("When true the collector captures automatically once settled and tag is stable. " +
             "When false the user must press Confirm to capture each pose.")]
    [SerializeField] bool _autoCapture = false;

    [Tooltip("When true, no waypoints are commanded. Position the robot manually with the " +
             "teach pendant, then press Confirm to capture each pose. " +
             "Click Finish (or call FinishManualSession) to run the solver.")]
    [SerializeField] bool _manualMode = false;

    [Tooltip("Timeout per waypoint in seconds. The waypoint is skipped if the robot does " +
             "not settle or the tag is not detected within this window. " +
             "Not used in manual mode.")]
    [SerializeField] float _waypointTimeoutSec = 15f;

    [Tooltip("Bypass real AprilTag detection and synthesise a tag pose from FK. " +
             "Lets you test the full capture->solver->save pipeline using only MoveIt joint states " +
             "without a camera feed. The recovered X will reflect _pseudoCamLocalOffset, not a real calibration.")]
    [SerializeField] bool _pseudoDetection = false;

    [Tooltip("Local-space offset of the fake camera from tool0 (metres). " +
             "Only used when _pseudoDetection is enabled.")]
    [SerializeField] Vector3 _pseudoCamLocalOffset = new Vector3(0.05f, 0f, 0.1f);

    [Tooltip("Fixed world-space position of the pseudo calibration tag (metres). " +
             "Place it somewhere the robot can 'see' across all waypoints. " +
             "Only used when _pseudoDetection is enabled.")]
    [SerializeField] Vector3 _pseudoTagWorldPos = new Vector3(0f, 6.3f, 0.4f);

    // State exposed to CalibrationHUD

    public enum State
    {
        Idle,
        WaypointApproach,
        WaitingForTag,
        ReadyToCapture,
        Captured,
        Solving,
        Done,
        Failed
    }

    public State CurrentState   { get; private set; } = State.Idle;
    public int   CurrentPose    { get; private set; } = 0;
    public int   TotalPoses     => Waypoints.Length;
    public int   CapturedCount  { get; private set; } = 0;
    public float ResidualDeg    { get; private set; } = float.NaN;
    public string StatusMessage { get; private set; } = "Idle";
    public bool  ManualMode     => _manualMode;
    public bool  PseudoDetection => _pseudoDetection;

    // Fired when the state machine transitions to ReadyToCapture so the HUD
    // can enable the Confirm button.
    public event Action OnReadyToCapture;

    // Fired when the full session completes (State.Done or State.Failed).
    public event Action<CalibrationResult> OnCalibrationComplete;

    // Waypoint table (joint angles in degrees).
    // Five clusters of three poses each. Within a cluster, the wrist executes a
    // rotation-rich motion to ensure |theta| > 5 deg. Poses are chosen to keep the
    // calibration target visible and avoid singularities near the home configuration
    // {0, -90, 90, -90, -90, 0}.

    static readonly float[][] Waypoints = new float[][]
    {
        // Cluster 1: shoulder pan sweep with elbow mid
        new float[] {  10f, -80f,  85f, -95f,  -85f,  10f },
        new float[] { -10f, -80f,  85f, -95f,  -85f, -10f },
        new float[] {   0f, -80f,  85f, -95f,  -85f,   0f },

        // Cluster 2: wrist tilt series (wrist_1 variation)
        new float[] {   0f, -75f,  80f,-100f,  -90f,   0f },
        new float[] {   0f, -85f,  80f, -85f,  -90f,   0f },
        new float[] {   0f, -80f,  80f, -92f,  -90f,   0f },

        // Cluster 3: wrist_2 roll variation (wrist_2 varies around -90)
        new float[] {   0f, -80f,  90f, -90f, -100f,   0f },
        new float[] {   0f, -80f,  90f, -90f,  -80f,   0f },
        new float[] {   0f, -80f,  90f, -90f,  -90f,  15f },

        // Cluster 4: combined shoulder and elbow variation
        new float[] {  15f, -85f,  95f, -88f,  -90f,  -5f },
        new float[] { -15f, -85f,  95f, -88f,  -90f,   5f },
        new float[] {   5f, -88f,  88f, -92f,  -90f,   0f },

        // Cluster 5: elevated elbow, wrist_3 roll variation
        new float[] {   0f, -70f, 100f, -90f,  -90f,  20f },
        new float[] {   0f, -70f, 100f, -90f,  -90f, -20f },
        new float[] {   0f, -70f, 100f, -90f,  -90f,   0f },
    };

    // Internal state

    readonly List<(Matrix4x4 A, Matrix4x4 B)> _pairs = new List<(Matrix4x4, Matrix4x4)>();

    // EEF and tag poses from the previous capture (needed to form relative pair)
    Matrix4x4 _prevEEF     = Matrix4x4.identity;
    Matrix4x4 _prevCamTag  = Matrix4x4.identity;
    bool      _hasPrev     = false;

    float _settleTimer       = 0f;
    int   _stableTagCounter  = 0;
    float _waypointTimer     = 0f;
    bool  _confirmRequested  = false;
    bool  _sessionActive     = false;
    bool  _savedManualControl = false;  // publisher.manualControlMode value before session

    readonly float[] _jointBuffer = new float[6];

    // Public interface

    /// Begin a new calibration session. Resets all collected data.
    public void StartCalibration()
    {
        if (_sessionActive)
        {
            Debug.LogWarning("[HandEyeCalibrationCollector] Session already active.");
            return;
        }

        _pairs.Clear();
        _hasPrev        = false;
        CapturedCount   = 0;
        CurrentPose     = 0;
        ResidualDeg     = float.NaN;
        _confirmRequested = false;
        _sessionActive   = true;

        // Disable manualControlMode so the /joint_states subscriber drives
        // the ArticulationBodies - this lets the digital twin follow the real
        // robot (teach pendant or ROS commands) during calibration.
        if (_publisher != null)
        {
            _savedManualControl = _publisher.manualControlMode;
            _publisher.manualControlMode = false;
        }

        if (_manualMode)
        {
            SetState(State.WaitingForTag);
            StatusMessage = "Manual: move robot, then press Confirm to capture (0 pairs so far)";
        }
        else
        {
            SetState(State.WaypointApproach);
            MoveToCurrentWaypoint();
            StatusMessage = "Moving to pose 1 of " + TotalPoses;
        }
        Debug.Log("[HandEyeCalibrationCollector] Calibration session started.");
    }

    /// Set manual vs. auto-waypoint mode.  Must be called before StartCalibration.
    public void SetManualMode(bool manual) { _manualMode = manual; }

    /// Finish a manual-mode session and run the Tsai solver on whatever pairs
    /// have been collected.  Requires at least HandEyeSolver.MinPairs pairs.
    public void FinishManualSession()
    {
        if (!_sessionActive || !_manualMode)
        {
            Debug.LogWarning("[HandEyeCalibrationCollector] FinishManualSession called outside of active manual session.");
            return;
        }
        if (_pairs.Count < HandEyeSolver.MinPairs)
        {
            StatusMessage = "Need at least " + HandEyeSolver.MinPairs + " pairs (have " + _pairs.Count + ")";
            Debug.LogWarning("[HandEyeCalibrationCollector] " + StatusMessage);
            return;
        }
        RunSolver();
    }

    /// Cancel the active session.
    public void CancelCalibration()
    {
        if (!_sessionActive) return;
        _sessionActive = false;
        StopAllCoroutines();
        RestoreManualControl();
        if (!_manualMode) _publisher?.MoveToHomePosition();
        SetState(State.Idle);
        StatusMessage = "Cancelled";
        Debug.Log("[HandEyeCalibrationCollector] Calibration cancelled.");
    }

    void RestoreManualControl()
    {
        if (_publisher != null)
            _publisher.manualControlMode = _savedManualControl;
    }

    /// Called by the UI Confirm button when auto-capture is disabled.
    public void ConfirmCapture()
    {
        if (CurrentState == State.ReadyToCapture)
            _confirmRequested = true;
    }

    // MonoBehaviour

    void Awake()
    {
        if (_publisher == null) _publisher = FindObjectOfType<UR3SourceDestinationPublisher>();
        if (_fkSolver  == null) _fkSolver  = FindObjectOfType<RobotFKSolver>();
        if (_detection == null) _detection = FindObjectOfType<Detection>();
    }

    void FixedUpdate()
    {
        if (!_sessionActive) return;

        _waypointTimer += Time.fixedDeltaTime;

        switch (CurrentState)
        {
            case State.WaypointApproach:
                TickApproach();
                break;

            case State.WaitingForTag:
                TickWaitForTag();
                break;

            case State.ReadyToCapture:
                TickReadyToCapture();
                break;
        }
    }

    // State machine ticks

    void TickApproach()
    {
        if (_waypointTimer > _waypointTimeoutSec)
        {
            Debug.LogWarning("[HandEyeCalibrationCollector] Pose " + (CurrentPose + 1) + " settle timeout - skipping.");
            AdvanceOrFinish();
            return;
        }

        if (!IsSettled())
        {
            _settleTimer = 0f;
            return;
        }

        _settleTimer += Time.fixedDeltaTime;
        if (_settleTimer >= _settleTime)
        {
            _settleTimer = 0f;
            _stableTagCounter = 0;
            SetState(State.WaitingForTag);
            StatusMessage = "Pose " + (CurrentPose + 1) + "/" + TotalPoses + "  - waiting for tag";
        }
    }

    void TickWaitForTag()
    {
        // In manual mode there is no per-pose timeout - the user takes as long as needed.
        if (!_manualMode && _waypointTimer > _waypointTimeoutSec)
        {
            Debug.LogWarning("[HandEyeCalibrationCollector] Pose " + (CurrentPose + 1) + " tag timeout - skipping.");
            AdvanceOrFinish();
            return;
        }

        if (GetTagCameraPose(out _))
            _stableTagCounter++;
        else
            _stableTagCounter = 0;

        if (_stableTagCounter >= _stableTagFrames)
        {
            SetState(State.ReadyToCapture);
            StatusMessage = "Pose " + (CurrentPose + 1) + "/" + TotalPoses + "  - ready  (confirm or auto)";
            OnReadyToCapture?.Invoke();
        }
    }

    void TickReadyToCapture()
    {
        bool trigger = _autoCapture || _confirmRequested;
        if (!trigger) return;

        _confirmRequested = false;
        TryCapture();
    }

    // Capture logic

    void TryCapture()
    {
        if (!GetTagCameraPose(out Matrix4x4 camTag))
        {
            Debug.LogWarning("[HandEyeCalibrationCollector] Tag lost at capture time for pose "
                + (CurrentPose + 1) + " - skipping.");
            AdvanceOrFinish();
            return;
        }

        Matrix4x4 eef = _fkSolver.GetEEFMatrix();

        if (_hasPrev)
        {
            // A_i = T_EEF_{i-1}^{-1} * T_EEF_i
            Matrix4x4 A = _prevEEF.inverse * eef;
            // B_i = T_cam_{i-1}^{-1} * T_cam_i
            Matrix4x4 B = _prevCamTag.inverse * camTag;
            _pairs.Add((A, B));
            CapturedCount++;
            Debug.Log("[HandEyeCalibrationCollector] Captured pair " + CapturedCount
                + " at pose " + (CurrentPose + 1));
        }
        else
        {
            Debug.Log("[HandEyeCalibrationCollector] Recorded initial pose (no pair yet).");
        }

        _prevEEF    = eef;
        _prevCamTag = camTag;
        _hasPrev    = true;

        SetState(State.Captured);
        StatusMessage = "Captured " + CapturedCount + " pair(s)";
        AdvanceOrFinish();
    }

    void AdvanceOrFinish()
    {
        if (_manualMode)
        {
            // Stay in session - reset counters and wait for the next manual capture.
            _waypointTimer    = 0f;
            _settleTimer      = 0f;
            _stableTagCounter = 0;
            SetState(State.WaitingForTag);
            StatusMessage = "Manual: " + CapturedCount + " pair(s) captured"
                + "  (need >= " + HandEyeSolver.MinPairs + ")  - move to next pose";
            return;
        }

        CurrentPose++;

        if (CurrentPose >= TotalPoses)
        {
            RunSolver();
            return;
        }

        _waypointTimer    = 0f;
        _settleTimer      = 0f;
        _stableTagCounter = 0;
        SetState(State.WaypointApproach);
        MoveToCurrentWaypoint();
        StatusMessage = "Moving to pose " + (CurrentPose + 1) + " of " + TotalPoses;
    }

    // Solver

    void RunSolver()
    {
        _sessionActive = false;
        RestoreManualControl();
        SetState(State.Solving);
        StatusMessage = "Solving...";

        if (_pairs.Count < HandEyeSolver.MinPairs)
        {
            Debug.LogError("[HandEyeCalibrationCollector] Not enough valid pairs to solve ("
                + _pairs.Count + " collected, need " + HandEyeSolver.MinPairs + ").");
            SetState(State.Failed);
            StatusMessage = "Failed - insufficient pairs (" + _pairs.Count + ")";
            OnCalibrationComplete?.Invoke(null);
            return;
        }

        Matrix4x4 X = HandEyeSolver.Solve(_pairs, out float residual);
        ResidualDeg  = residual;

        CalibrationResult result = new CalibrationResult
        {
            timestamp  = DateTime.UtcNow.ToString("yyyy-MM-ddTHH:mm:ssZ"),
            pairsUsed  = _pairs.Count,
            residualDeg = residual
        };
        result.FromMatrix4x4(X);
        string savePath = result.Save();

        SetState(State.Done);
        StatusMessage = savePath != null
            ? "Done  residual=" + residual.ToString("F2") + " deg  pairs=" + _pairs.Count
            : "Done (save failed)  residual=" + residual.ToString("F2") + " deg";

        Vector3 t   = result.Translation;
        Vector3 eul = result.Rotation.eulerAngles;
        Debug.Log("[HandEyeCalibrationCollector] Solved T_tool0_to_camera:"
            + "\n  t = (" + t.x.ToString("+0.0000;-0.0000")
            + ", "        + t.y.ToString("+0.0000;-0.0000")
            + ", "        + t.z.ToString("+0.0000;-0.0000") + ") m"
            + "\n  r = P" + eul.x.ToString("F1")
            + " Y"        + eul.y.ToString("F1")
            + " R"        + eul.z.ToString("F1") + " deg"
            + "\n  residual = " + residual.ToString("F3") + " deg"
            + "  pairs = "    + _pairs.Count);

        if (!_manualMode) _publisher?.MoveToHomePosition();
        ApplyCalibrationToScene(result);
        OnCalibrationComplete?.Invoke(result);
    }

    // Apply the solved transform back to the scene camera mount.
    /// Moves tool0CameraTransform to the pose stored in <paramref name="result"/>.
    /// T_tool0_to_camera is the local-space offset of the camera from tool0,
    /// so we set localPosition and localRotation on tool0CameraTransform.
    /// Call this after Load JSON to update the digital-twin camera mount.
    public void ApplyCalibrationToScene(CalibrationResult result)
    {
        if (result == null || _fkSolver == null || _fkSolver.tool0CameraTransform == null)
            return;

        Matrix4x4 X   = result.ToMatrix4x4();
        Vector3    pos = X.GetColumn(3);
        Quaternion rot = X.rotation;

        _fkSolver.tool0CameraTransform.localPosition = pos;
        _fkSolver.tool0CameraTransform.localRotation = rot;

        Debug.Log("[HandEyeCalibrationCollector] Camera mount updated:"
            + "  t=(" + pos.x.ToString("+0.0000;-0.0000")
            + ", "   + pos.y.ToString("+0.0000;-0.0000")
            + ", "   + pos.z.ToString("+0.0000;-0.0000") + ") m"
            + "  residual=" + result.residualDeg.ToString("F2") + " deg");
    }

    // Helpers

    void MoveToCurrentWaypoint()
    {
        if (CurrentPose >= TotalPoses) return;
        float[] wp = Waypoints[CurrentPose];
        for (int i = 0; i < 6; i++)
            _publisher.SetJointAngleLocally(i, wp[i]);
    }

    bool IsSettled()
    {
        if (!_publisher.GetActualJointAnglesInto(_jointBuffer)) return false;
        float[] target = Waypoints[CurrentPose];
        for (int i = 0; i < 6; i++)
        {
            float delta = Mathf.Abs(Mathf.DeltaAngle(_jointBuffer[i], target[i]));
            if (delta > _settleThresholdDeg) return false;
        }
        return true;
    }

    // Returns the camera-space pose of the calibration tag as a 4x4 matrix.
    // Returns false when the tag is not detected (or pseudo detection is off and Detection is null).
    bool GetTagCameraPose(out Matrix4x4 pose)
    {
        // Pseudo mode: synthesise tag-in-camera-space from FK.
        // B_i = inv(cam_{i-1}) * cam_i still varies with robot motion, so the Tsai solver
        // receives a non-degenerate pair set. The recovered X reflects _pseudoCamLocalOffset,
        // not a real calibration - useful only for verifying the pipeline wiring.
        if (_pseudoDetection && _fkSolver != null)
        {
            Matrix4x4 eef      = _fkSolver.GetEEFMatrix();
            Matrix4x4 camWorld = eef * Matrix4x4.TRS(_pseudoCamLocalOffset, Quaternion.identity, Vector3.one);
            Matrix4x4 tagWorld = Matrix4x4.TRS(_pseudoTagWorldPos, Quaternion.identity, Vector3.one);
            pose = camWorld.inverse * tagWorld;
            return true;
        }

        if (_detection?.DetectedTags == null) { pose = Matrix4x4.identity; return false; }

        foreach (var tag in _detection.DetectedTags)
        {
            if (tag.ID != _targetTagId) continue;
            Pose p = PoseEstimation.GetCameraPose(tag);
            pose   = Matrix4x4.TRS(p.position, p.rotation, Vector3.one);
            return true;
        }

        pose = Matrix4x4.identity;
        return false;
    }

    void SetState(State s) => CurrentState = s;
}
