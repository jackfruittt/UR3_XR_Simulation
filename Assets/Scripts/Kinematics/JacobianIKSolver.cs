// Author: Jackson Russell

using UnityEngine;

/// RMRC + Gradient Descent IK controller for the UR3e.
///
/// References:
///  Whitney (1969) - Resolved Motion Rate Control
///  Yoshikawa (1985) - Manipulability of Robotic Mechanisms
///  https://roboticsknowledgebase.com/wiki/planning/resolved-rates/
///
/// Notes:
///  RMRC:      q_dot = J_pinv * v_des,   v_des = [Kp * dp; Ko * dphi]  (6-vectors)
///  DLS:       J_pinv = (J^T * J + lambda^2 * I)^-1 * J^T
///  GD:        q_dot_GD = alpha * J^T * v_des  (no inversion, bounded)
///  Blend:     w = |det(J)|,  beta = w / (w + wEps)
///             q_dot = beta * q_dot_DLS + (1 - beta) * q_dot_GD
///  Integrate: q_new = q_actual + q_dot * dt  (written to ArticulationBody drives)

public class JacobianIKSolver : MonoBehaviour
{
    // Inspector
    [Header("References")]
    public UR3SourceDestinationPublisher publisher;
    public RobotFKSolver                 fkSolver;
    public EEFTargetController           targetController;
    [Tooltip("Optional: artificial potential field collision avoidance. " +
             "Attach a SelfCollisionAvoider component and assign here to add " +
             "q_dot_rep after the RMRC/GD blend.")]
    public SelfCollisionAvoider          potentialField;

    [Header("RMRC Gains")]
    [Tooltip("Proportional gain on position error (m/s per m).")]
    public float posGain = 3f;

    [Tooltip("Proportional gain on orientation error (rad/s per rad).")]
    public float oriGain = 3f;

    [Tooltip("Scales orientation rows of J. 0 = position-only, raise for tighter wrist control.")]
    [Range(0f, 1f)]
    public float orientationWeight = 0.2f;

    [Header("DLS Parameters")]
    [Tooltip("DLS damping lambda. Prevents blow-up near singularities (typical: 0.05-0.3).")]
    public float lambdaDLS = 0.1f;

    [Header("Gradient Descent Blend")]
    [Tooltip("Manipulability threshold. Below this, solver blends toward gradient descent.")]
    public float wEps = 0.01f;

    [Tooltip("GD step size near singularity. Scales J^T * v_des (0.1-1.0).")]
    public float stepGD = 0.5f;

    [Tooltip("Beta threshold below which orientation rows are zeroed; gives the solver freedom to escape bad configs.")]
    [Range(0f, 0.5f)]
    public float oriDropThreshold = 0.2f;

    [Header("Safety")]
    [Tooltip("Soft per-joint velocity ceiling (rad/s). The whole q_dot vector is scaled " +
             "proportionally when any joint would exceed this, preserving motion direction. " +
             "Hard physical limit is pi (~3.14) rad/s; default 0.8*pi keeps a safety margin.")]
    public float maxJointVelRad = 0.8f * Mathf.PI;

    [Tooltip("Position error below which RMRC idles (metres). Prevents micro-jitter at goal.")]
    public float positionTolerance = 0.001f;

    [Tooltip("Orientation error (radians, weighted) below which RMRC idles.")]
    public float orientationTolerance = 0.01f;

    [Header("Control")]
    public bool solverEnabled = true;

    [Tooltip("Log singularity warnings to Console.")]
    public bool logSingularityWarnings = true;

    [Header("Singularity Escape")]
    [Tooltip("Beta threshold for stuck detection; triggers auto-home after singularityStuckTime seconds.")]
    [Range(0f, 0.15f)]
    public float escapeThreshold = 0.05f;

    [Tooltip("Seconds beta must stay below escapeThreshold before auto-homing triggers.")]
    public float singularityStuckTime = 1.5f;

    [Tooltip("Position error (metres) above which the no-progress timer counts. " +
             "If error stays above this for noProgressTimeout seconds the arm homes.")]
    public float noProgressErrorThreshold = 0.08f;

    [Tooltip("Seconds the position error must stay above noProgressErrorThreshold " +
             "before auto-homing triggers. Catches unreachable targets and locked configurations.")]
    public float noProgressTimeout = 4f;

    [Tooltip("Maximum seconds to wait for the arm to reach home before resuming anyway.")]
    public float homingTimeout = 5f;

    [Tooltip("Per-joint tolerance (degrees) for declaring that homing is complete.")]
    public float homeArrivalTolerance = 5f;

    [Tooltip("Settle pause (seconds) after arriving at home before IK resumes.")]
    public float homeResumeDelay = 0.5f;

    [Tooltip("Delay after first grab before the singularity-stuck timer activates.")]
    public float startupGracePeriod = 3.0f;

    // Singularity escape state machine
    public enum SolverState { Tracking, EscapingToHome, ResumeDelay }

    /// Current phase of the singularity escape state machine.
    public SolverState CurrentSolverState { get; private set; } = SolverState.Tracking;

    private float _stuckTimer        = 0f;   // time spent with beta < escapeThreshold
    private float _noProgressTimer   = 0f;   // time spent with error > noProgressErrorThreshold
    private float _escapeTimer       = 0f;   // time spent waiting for the arm to reach home
    private float _resumeTimer       = 0f;   // countdown before IK resumes after homing
    private float _graceTimer        = -1f;  // countdown for startup grace; -1 = not yet started
    private bool  _wasGrabbed        = false;

    /// Most recently classified singularity type.
    public SingularityChecker.SingularityType LastSingularityType { get; private set; }
        = SingularityChecker.SingularityType.None;

    /// Manipulability |det(J)| at last step - exposed for the HUD.
    public float LastManipulability { get; private set; }

    /// DLS/GD blend factor beta at last step (1 = pure DLS, 0 = pure GD).
    public float LastBlendFactor { get; private set; }

    /// Per-joint velocity (rad/s) after proportional scaling, last step.
    /// Index matches joint order. Updated every FixedUpdate including homing.
    /// External readers always receive the same array reference; data is overwritten in place.
    private readonly float[] _lastJointVelRad = new float[6];
    public float[] LastJointVelocitiesRad => _lastJointVelRad;

    // Pre-allocated working arrays - eliminates heap allocations every FixedUpdate at 50 Hz.
    private readonly float[]  _v        = new float[6];
    private readonly float[]  _qRad     = new float[6];
    private readonly float[,] _J        = new float[6, 6];
    private readonly float[,] _A        = new float[6, 6];
    private readonly float[]  _b        = new float[6];
    private readonly float[]  _qdotDLS  = new float[6];
    private readonly float[]  _qdotGD   = new float[6];
    private readonly float[]  _qdot     = new float[6];
    private readonly float[]  _goalDeg  = new float[6];
    private readonly float[]  _qdotHome = new float[6];
    // Gaussian elimination scratch: [6,7] augmented matrix.
    private readonly float[,] _gaussM   = new float[6, 7];

    // Pre-allocated actual joint angle buffer.
    // GetActualJointAnglesInto() fills this in-place - replaces GetActualJointAngles()
    // which allocated new float[6] every FixedUpdate (50 heap allocs/s).
    private readonly float[] _actualDeg = new float[6];

    // Cached squared tolerances - avoids sqrt in sqrMagnitude comparisons each frame.
    private float _posToleranceSq;
    private float _noProgressSq;

    void Start()
    {
        _posToleranceSq = positionTolerance * positionTolerance;
        _noProgressSq   = noProgressErrorThreshold * noProgressErrorThreshold;
    }

    // FixedUpdate - RMRC core loop
    void FixedUpdate()
    {
        if (!solverEnabled) return;
        if (publisher == null || fkSolver == null || targetController == null) return;

        ArticulationBody[] bodies = publisher.JointBodies;
        if (bodies == null) return;

        float dt    = Time.fixedDeltaTime;
        float invDt = 1f / dt;   // cached once - used in homing velocity calculation below

        // Singularity escape state machine:
        //  EscapingToHome - velocity-limited stepping toward home each FixedUpdate.
        //  ResumeDelay    - brief settle after homing before IK restarts.
        if (CurrentSolverState == SolverState.EscapingToHome)
        {
            _escapeTimer += dt;

            float[] homePos  = publisher.GetHomePosition();
            bool    gotActual = publisher.GetActualJointAnglesInto(_actualDeg);

            if (homePos != null && gotActual)
            {
                // Desired velocity: "close all remaining error this step".
                // LimitVelocities scales the whole vector down proportionally if any
                // joint would exceed the cap - same rule as normal tracking, no separate gain.
                for (int j = 0; j < 6; j++)
                    _qdotHome[j] = Mathf.DeltaAngle(_actualDeg[j], homePos[j])
                                   * Mathf.Deg2Rad * invDt;

                LimitVelocities(_qdotHome);
                System.Array.Copy(_qdotHome, _lastJointVelRad, 6);

                for (int j = 0; j < 6; j++)
                    publisher.SetJointAngleLocally(
                        j, _actualDeg[j] + _qdotHome[j] * Mathf.Rad2Deg * dt);
            }

            if (IsAtHome() || _escapeTimer >= homingTimeout)
            {
                CurrentSolverState = SolverState.ResumeDelay;
                _resumeTimer       = homeResumeDelay;
                _escapeTimer       = 0f;
                if (logSingularityWarnings)
                    Debug.Log("[RMRC] Homing complete - settling before resuming IK.");
            }
            return;
        }

        if (CurrentSolverState == SolverState.ResumeDelay)
        {
            _resumeTimer -= dt;
            if (_resumeTimer <= 0f)
            {
                CurrentSolverState = SolverState.Tracking;
                _stuckTimer        = 0f;
                if (logSingularityWarnings)
                    Debug.Log("[RMRC] Singularity escape complete - IK resumed.");
            }
            return;   // brief settle pause after homing
        }

        // Stay idle until the player has grabbed the orb at least once.
        // Prevents the arm charging across the scene on startup.
        if (!targetController.HasBeenGrabbed) return;

        // Start the grace timer.
        if (!_wasGrabbed)
        {
            _wasGrabbed = true;
            _graceTimer = startupGracePeriod;
            _stuckTimer = 0f;
        }
        if (_graceTimer > 0f) _graceTimer -= dt;

        // 1. Task-space error - position and orientation delta in world space.
        Vector3 posErr = targetController.TargetPosition - fkSolver.GetEEFPosition();

        Quaternion errQ = targetController.TargetRotation
                          * Quaternion.Inverse(fkSolver.GetEEFRotation());
        errQ.ToAngleAxis(out float errAngle, out Vector3 errAxis);
        errAngle *= Mathf.Deg2Rad;   // Unity ToAngleAxis returns degrees; convert to radians
        if (errAngle > Mathf.PI) errAngle -= 2f * Mathf.PI;   // wrap to [-pi, pi] shortest path
        if (errAxis.sqrMagnitude < 1e-6f) errAxis = Vector3.up;

        float oriMag = Mathf.Abs(errAngle) * orientationWeight;

        // sqrMagnitude avoids sqrt - compare against pre-squared tolerance (cached in Start).
        if (posErr.sqrMagnitude < _posToleranceSq && oriMag < orientationTolerance)
            return;   // at goal, nothing to do

        // 2. Desired EEF velocity: v = [Kp*dp; Ko*dphi*orientationWeight].
        _v[0] = posErr.x * posGain;
        _v[1] = posErr.y * posGain;
        _v[2] = posErr.z * posGain;
        _v[3] = errAxis.x * errAngle * oriGain * orientationWeight;
        _v[4] = errAxis.y * errAngle * oriGain * orientationWeight;
        _v[5] = errAxis.z * errAngle * oriGain * orientationWeight;

        // Near-singular: drop orientation rows -> position-only task with null-space freedom.
        // LastBlendFactor is one FixedUpdate stale - acceptable at 50 Hz.
        if (LastBlendFactor < oriDropThreshold)
        {
            _v[3] = 0f;
            _v[4] = 0f;
            _v[5] = 0f;
        }

        // 3. Geometric Jacobian J (6x6).
        BuildJacobian(_J, bodies, fkSolver.GetEEFPosition());

        // 4. Singularity evaluation - single trig pass returns det(J) and type together.
        bool  gotDeg  = publisher.GetActualJointAnglesInto(_actualDeg);
        float w_manip = 0f;

        if (gotDeg)
        {
            for (int i = 0; i < 6; i++) _qRad[i] = _actualDeg[i] * Mathf.Deg2Rad;

            SingularityChecker.EvalResult eval = SingularityChecker.Evaluate(_qRad);
            w_manip             = Mathf.Abs(eval.Determinant);
            LastManipulability  = w_manip;
            LastSingularityType = eval.Type;

            if (LastSingularityType != SingularityChecker.SingularityType.None
                && logSingularityWarnings)
                Debug.LogWarning($"[RMRC] Singularity: {LastSingularityType}  " +
                                 $"w={w_manip:F5}");
        }
        else
        {
            LastManipulability  = 0f;
            LastSingularityType = SingularityChecker.SingularityType.None;
        }

        // 5. Blend factor: beta = w / (w + wEps).
        // Approaches 1 when well-conditioned (DLS), 0 near singular (GD).
        float beta = w_manip / (w_manip + wEps);
        LastBlendFactor = beta;

        // Singularity escape: home if stuck below escapeThreshold.
        if (beta < escapeThreshold && _graceTimer <= 0f)
        {
            _stuckTimer += dt;
            if (_stuckTimer >= singularityStuckTime)
            {
                TriggerEscapeToHome($"singularity stuck {_stuckTimer:F1}s, type={LastSingularityType}");
                return;
            }
        }
        else
        {
            _stuckTimer = 0f;   // arm has self-recovered; reset the stuck timer
        }

        // No-progress escape: home if position error has been large for too long.
        // Catches unreachable targets and locked non-singular configurations.
        if (_graceTimer <= 0f)
        {
            // sqrMagnitude avoids sqrt - compare against pre-squared threshold (cached in Start).
            if (posErr.sqrMagnitude > _noProgressSq)
            {
                _noProgressTimer += dt;
                if (_noProgressTimer >= noProgressTimeout)
                {
                    TriggerEscapeToHome(
                        $"no progress for {_noProgressTimer:F1}s, " +
                        $"err={Mathf.Sqrt(posErr.sqrMagnitude) * 1000f:F0}mm");
                    return;
                }
            }
            else
            {
                _noProgressTimer = 0f;
            }
        }

        // 6a. DLS: q_dot = (J^T J + lambda^2 * I)^-1 J^T v
        bool dlsValid = false;
        if (beta > 0.01f)
        {
            float lambda2 = lambdaDLS * lambdaDLS;

            // J^T*J is symmetric: A[r,c] == A[c,r].
            // Only compute the upper triangle (21 unique entries), then mirror into lower.
            for (int r = 0; r < 6; r++)
            {
                for (int c = r; c < 6; c++)
                {
                    float s = 0f;
                    for (int k = 0; k < 6; k++) s += _J[k, r] * _J[k, c];
                    _A[r, c] = s;
                    _A[c, r] = s;   // mirror lower triangle
                }
                _A[r, r] += lambda2;
            }

            for (int r = 0; r < 6; r++)
            {
                float s = 0f;
                for (int k = 0; k < 6; k++) s += _J[k, r] * _v[k];
                _b[r] = s;
            }

            dlsValid = GaussianSolve(_A, _b, _gaussM, _qdotDLS);
        }

        // 6b. GD: q_dot = alpha * J^T * v
        for (int j = 0; j < 6; j++)
        {
            float s = 0f;
            for (int k = 0; k < 6; k++) s += _J[k, j] * _v[k];   // J^T[j,k] == J[k,j]
            _qdotGD[j] = s * stepGD;
        }

        // 7. Blend the DLS and GD contributions using beta.
        for (int j = 0; j < 6; j++)
        {
            float dls = dlsValid ? _qdotDLS[j] : 0f;
            _qdot[j] = beta * dls + (1f - beta) * _qdotGD[j];
        }

        // 8. Limit the RMRC/GD task velocities BEFORE adding repulsion.
        //    Proportional scaling here only affects the task motion, not the APF correction.
        LimitVelocities(_qdot);

        // 8b. Potential field: add repulsive correction AFTER task limiting so the
        //     proportional rescale inside LimitVelocities cannot shrink task motion.
        //     AccumulateRepulsiveQdot already clamps each joint to maxRepulsiveVelRad;
        //     a final per-joint hard clamp below catches any combined excess.
        potentialField?.AccumulateRepulsiveQdot(_qdot);
        for (int j = 0; j < 6; j++)
            _qdot[j] = Mathf.Clamp(_qdot[j], -Mathf.PI, Mathf.PI);
        System.Array.Copy(_qdot, _lastJointVelRad, 6);

        for (int j = 0; j < 6; j++)
        {
            float baseDeg = gotDeg ? _actualDeg[j] : 0f;
            _goalDeg[j] = baseDeg + _qdot[j] * Mathf.Rad2Deg * dt;
        }

        // 9. Push the new targets directly to the ArticulationBody drives.
        //    RMRC produces smooth, velocity-limited increments each step
        for (int j = 0; j < 6; j++)
            publisher.SetJointAngleLocally(j, _goalDeg[j]);
    }

    // Centralised escape trigger - sets state and logs once.
    void TriggerEscapeToHome(string reason)
    {
        CurrentSolverState = SolverState.EscapingToHome;
        _escapeTimer       = 0f;
        _stuckTimer        = 0f;
        _noProgressTimer   = 0f;
        if (logSingularityWarnings)
            Debug.LogWarning($"[RMRC] Homing triggered: {reason}.");
    }

    // Returns true when all joints are within homeArrivalTolerance degrees of home.
    bool IsAtHome()
    {
        float[] home = publisher.GetHomePosition();
        if (home == null || !publisher.GetActualJointAnglesInto(_actualDeg)) return false;
        for (int i = 0; i < 6; i++)
            if (Mathf.Abs(Mathf.DeltaAngle(_actualDeg[i], home[i])) > homeArrivalTolerance)
                return false;
        return true;
    }

    /// Proportionally scales qdot (rad/s) so no joint exceeds the configured soft ceiling
    /// (maxJointVelRad) and none exceeds the absolute hard limit of pi rad/s.
    /// Scaling is uniform across all joints, preserving the motion direction.
    void LimitVelocities(float[] qdot)
    {
        float soft = Mathf.Min(maxJointVelRad, Mathf.PI);
        float peak = 0f;
        for (int j = 0; j < qdot.Length; j++)
            if (Mathf.Abs(qdot[j]) > peak) peak = Mathf.Abs(qdot[j]);

        if (peak > soft)
        {
            float scale = soft / peak;
            for (int j = 0; j < qdot.Length; j++) qdot[j] *= scale;
        }

        // Hard clamp - catches any residual float imprecision after scaling.
        for (int j = 0; j < qdot.Length; j++)
            qdot[j] = Mathf.Clamp(qdot[j], -Mathf.PI, Mathf.PI);
    }

    // Geometric Jacobian J (6x6) - writes into pre-allocated buffer, no heap allocation.
    static void BuildJacobian(float[,] J, ArticulationBody[] bodies, Vector3 eefPos)
    {
        for (int i = 0; i < 6; i++)
        {
            if (bodies[i] == null) continue;

            // Joint axis in world space. The URDF importer maps the joint axis to local X.
            Vector3 z = bodies[i].transform.rotation
                        * bodies[i].anchorRotation
                        * Vector3.right;

            Vector3 r  = eefPos - bodies[i].transform.position;
            Vector3 tc = Vector3.Cross(z, r);

            J[0, i] = tc.x;   J[1, i] = tc.y;   J[2, i] = tc.z;
            J[3, i] = z.x;    J[4, i] = z.y;     J[5, i] = z.z;
        }
    }

    // 6x6 Gaussian elimination with partial pivoting.
    // Writes solution into pre-allocated result[]. Returns false on numerical singularity (pivot < 1e-10).
    // M is a pre-allocated [6,7] augmented-matrix scratch buffer.
    static bool GaussianSolve(float[,] A, float[] b, float[,] M, float[] result)
    {
        const int N = 6;
        for (int r = 0; r < N; r++)
        {
            for (int c = 0; c < N; c++) M[r, c] = A[r, c];
            M[r, N] = b[r];
        }

        for (int col = 0; col < N; col++)
        {
            int   pivot  = col;
            float maxVal = Mathf.Abs(M[col, col]);
            for (int row = col + 1; row < N; row++)
            {
                float v = Mathf.Abs(M[row, col]);
                if (v > maxVal) { maxVal = v; pivot = row; }
            }
            if (maxVal < 1e-10f) return false;

            if (pivot != col)
                for (int c = 0; c <= N; c++)
                {
                    float tmp = M[col, c]; M[col, c] = M[pivot, c]; M[pivot, c] = tmp;
                }

            float inv = 1f / M[col, col];
            for (int row = col + 1; row < N; row++)
            {
                float f = M[row, col] * inv;
                for (int c = col; c <= N; c++) M[row, c] -= f * M[col, c];
            }
        }

        // Back-substitution writes directly into result[] (filled high-to-low, safe in-place).
        for (int row = N - 1; row >= 0; row--)
        {
            float s = M[row, N];
            for (int c = row + 1; c < N; c++) s -= M[row, c] * result[c];
            result[row] = s / M[row, row];
        }
        return true;
    }
}

