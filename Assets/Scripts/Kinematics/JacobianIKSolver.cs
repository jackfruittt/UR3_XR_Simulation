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

    [Tooltip("GD step size near singularity. Scales J^T·v_des (0.1-1.0).")]
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
    private float _resumeTimer    = 0f;   // countdown before IK resumes after homing
    private float _graceTimer     = -1f;  // countdown for startup grace; -1 = not yet started
    private bool  _wasGrabbed     = false;

    /// Most recently classified singularity type.
    public SingularityChecker.SingularityType LastSingularityType { get; private set; }
        = SingularityChecker.SingularityType.None;

    /// Manipulability |det(J)| at last step - exposed for the HUD.
    public float LastManipulability { get; private set; }

    /// DLS/GD blend factor beta at last step (1 = pure DLS, 0 = pure GD).
    public float LastBlendFactor { get; private set; }

    /// Per-joint velocity (rad/s) after proportional scaling, last step.
    /// Index matches joint order. Updated every FixedUpdate including homing.
    public float[] LastJointVelocitiesRad { get; private set; } = new float[6];

    // FixedUpdate - RMRC core loop
    void FixedUpdate()
    {
        if (!solverEnabled) return;
        if (publisher == null || fkSolver == null || targetController == null) return;

        ArticulationBody[] bodies = publisher.JointBodies;
        if (bodies == null) return;

        float dt = Time.fixedDeltaTime;

        // Singularity escape state machine:
        //  EscapingToHome - velocity-limited stepping toward home each FixedUpdate.
        //  ResumeDelay    - brief settle after homing before IK restarts.
        if (CurrentSolverState == SolverState.EscapingToHome)
        {
            _escapeTimer += dt;

            float[] homePos = publisher.GetHomePosition();
            float[] actual  = publisher.GetActualJointAngles();
            if (homePos != null && actual != null)
            {
                // Desired velocity: "close all remaining error this step".
                // LimitVelocities scales the whole vector down proportionally if any
                // joint would exceed the cap — same rule as normal tracking, no separate gain.
                float[] qdotHome = new float[6];
                for (int j = 0; j < 6; j++)
                    qdotHome[j] = Mathf.DeltaAngle(actual[j], homePos[j]) * Mathf.Deg2Rad / dt;

                LimitVelocities(qdotHome);
                LastJointVelocitiesRad = qdotHome;

                for (int j = 0; j < 6; j++)
                    publisher.SetJointAngleLocally(j, actual[j] + qdotHome[j] * Mathf.Rad2Deg * dt);
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

        // Normal tracking path (SolverState.Tracking)

        // Stay idle until the player has grabbed the orb at least once.
        // Prevents the arm charging across the scene on startup.
        if (!targetController.HasBeenGrabbed) return;

        // Start the grace timer.
        if (!_wasGrabbed)
        {
            _wasGrabbed   = true;
            _graceTimer   = startupGracePeriod;
            _stuckTimer   = 0f;
        }
        if (_graceTimer > 0f)
            _graceTimer -= dt;

        // 1. Task-space error - position and orientation delta in world space.
        Vector3 posErr = targetController.TargetPosition - fkSolver.GetEEFPosition();

        Quaternion errQ = targetController.TargetRotation
                          * Quaternion.Inverse(fkSolver.GetEEFRotation());
        errQ.ToAngleAxis(out float errAngle, out Vector3 errAxis);
        errAngle *= Mathf.Deg2Rad;   // Unity ToAngleAxis returns degrees; convert to radians
        if (errAngle > Mathf.PI) errAngle -= 2f * Mathf.PI;   // wrap to [-pi, pi] shortest path
        if (errAxis.sqrMagnitude < 1e-6f) errAxis = Vector3.up;

        float oriMag = Mathf.Abs(errAngle) * orientationWeight;

        if (posErr.magnitude < positionTolerance && oriMag < orientationTolerance)
            return;   // at goal, nothing to do

        // 2. Desired EEF velocity: v = [Kp*dp; Ko*dphi*orientationWeight].
        float[] v = new float[6]
        {
            posErr.x * posGain,
            posErr.y * posGain,
            posErr.z * posGain,
            errAxis.x * errAngle * oriGain * orientationWeight,
            errAxis.y * errAngle * oriGain * orientationWeight,
            errAxis.z * errAngle * oriGain * orientationWeight,
        };

        // Near-singular: drop orientation rows -> position-only task with null-space freedom.
        // LastBlendFactor is one FixedUpdate stale - acceptable at 50 Hz.
        if (LastBlendFactor < oriDropThreshold)
        {
            v[3] = 0f;
            v[4] = 0f;
            v[5] = 0f;
        }

        // 3. Geometric Jacobian J (6x6).
        float[,] J = BuildJacobian(bodies, fkSolver.GetEEFPosition());

        // 4. Manipulability and singularity type (closed-form det).
        float[] actualDeg = publisher.GetActualJointAngles();
        float   w_manip   = 0f;

        if (actualDeg != null)
        {
            float[] qRad = new float[6];
            for (int i = 0; i < 6; i++) qRad[i] = actualDeg[i] * Mathf.Deg2Rad;

            w_manip              = Mathf.Abs(SingularityChecker.JacobianDeterminant(qRad));
            LastManipulability   = w_manip;
            LastSingularityType  = SingularityChecker.Classify(qRad);

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
            if (posErr.magnitude > noProgressErrorThreshold)
            {
                _noProgressTimer += dt;
                if (_noProgressTimer >= noProgressTimeout)
                {
                    TriggerEscapeToHome($"no progress for {_noProgressTimer:F1}s, err={posErr.magnitude * 1000f:F0}mm");
                    return;
                }
            }
            else
            {
                _noProgressTimer = 0f;
            }
        }

        // 6a. DLS: q_dot = (J^T J + lambda^2 * I)^-1 J^T v
        float[] qdot_DLS = null;
        if (beta > 0.01f)
        {
            float lambda2 = lambdaDLS * lambdaDLS;
            float[,] A = new float[6, 6];
            for (int r = 0; r < 6; r++)
            {
                for (int c = 0; c < 6; c++)
                {
                    float s = 0f;
                    for (int k = 0; k < 6; k++) s += J[k, r] * J[k, c];
                    A[r, c] = s;
                }
                A[r, r] += lambda2;
            }

            float[] b = new float[6];
            for (int r = 0; r < 6; r++)
            {
                float s = 0f;
                for (int k = 0; k < 6; k++) s += J[k, r] * v[k];
                b[r] = s;
            }

            qdot_DLS = GaussianSolve(A, b);   // null on numerical failure
        }

        // 6b. GD: q_dot = alpha · J^T v
        float[] qdot_GD = new float[6];
        for (int j = 0; j < 6; j++)
        {
            float s = 0f;
            for (int k = 0; k < 6; k++) s += J[k, j] * v[k];   // J^T[j,k] == J[k,j]
            qdot_GD[j] = s * stepGD;
        }

        // 7. Blend the DLS and GD contributions using beta.
        float[] qdot = new float[6];
        for (int j = 0; j < 6; j++)
        {
            float dls = (qdot_DLS != null) ? qdot_DLS[j] : 0f;
            qdot[j] = beta * dls + (1f - beta) * qdot_GD[j];
        }

        // 8. Apply velocity limit, then integrate: q_new = q_actual + q_dot * dt
        LimitVelocities(qdot);
        LastJointVelocitiesRad = qdot;

        float[] goalDeg = new float[6];
        for (int j = 0; j < 6; j++)
        {
            float baseDeg = (actualDeg != null) ? actualDeg[j] : 0f;
            goalDeg[j] = baseDeg + qdot[j] * Mathf.Rad2Deg * dt;
        }

        // 9. Push the new targets directly to the ArticulationBody drives.
        //    RMRC produces smooth, velocity-limited increments each step
        for (int j = 0; j < 6; j++)
            publisher.SetJointAngleLocally(j, goalDeg[j]);
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
        float[] home   = publisher.GetHomePosition();
        float[] actual = publisher.GetActualJointAngles();
        if (home == null || actual == null) return false;
        for (int i = 0; i < 6; i++)
            if (Mathf.Abs(Mathf.DeltaAngle(actual[i], home[i])) > homeArrivalTolerance)
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

    // Geometric Jacobian J (6x6)
    static float[,] BuildJacobian(ArticulationBody[] bodies, Vector3 eefPos)
    {
        float[,] J = new float[6, 6];
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
        return J;
    }

    // 6x6 Gaussian elimination with partial pivoting
    // Returns null on numerical singularity (pivot < 1e-10).
    static float[] GaussianSolve(float[,] A, float[] b)
    {
        const int N = 6;
        float[,] M = new float[N, N + 1];
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
            if (maxVal < 1e-10f) return null;

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

        float[] x = new float[N];
        for (int row = N - 1; row >= 0; row--)
        {
            float s = M[row, N];
            for (int c = row + 1; c < N; c++) s -= M[row, c] * x[c];
            x[row] = s / M[row, row];
        }
        return x;
    }
}

