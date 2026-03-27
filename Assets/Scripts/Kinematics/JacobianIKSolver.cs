using UnityEngine;

/// Damped Least Squares (DLS) Jacobian IK solver for the UR3e.
///
/// Since the robot has exactly 6 joints and we control 6 task-space DOF
/// (3 position + 3 orientation), J is square (6x6). The DLS normal equations
/// reduce to a 6x6 linear system solved by Gaussian elimination - no
/// pseudo-inverse or SVD required.
///
/// The geometric Jacobian is computed analytically from the ArticulationBody
/// anchor axes and current link transforms each FixedUpdate step.
///
/// DLS:
///   (J^T J + dampingLambda^2 * I) dTheta = J^T dx
/// where dampingLambda is the damping factor that prevents blowup near singularities.
public class JacobianIKSolver : MonoBehaviour
{
    [Header("References")]
    public UR3SourceDestinationPublisher publisher;
    public RobotFKSolver fkSolver;
    public EEFTargetController targetController;

    [Header("DLS Parameters")]
    [Tooltip("Damping factor (lambda). Increase (e.g. 0.5) to stabilise near singularities.")]
    public float dampingLambda = 0.1f;
    [Tooltip("Max joint angle change applied per physics step (degrees).")]
    public float maxStepDeg = 3f;
    [Tooltip("Position error below which the solver stops iterating (metres).")]
    public float positionTolerance = 0.001f;
    [Tooltip("Orientation error below which the solver stops iterating (radians).")]
    public float orientationTolerance = 0.01f;
    [Tooltip("Scales orientation rows of J relative to position rows. Reduce if wrist flips.")]
    [Range(0f, 1f)]
    public float orientationWeight = 0.3f;

    [Header("Control")]
    public bool solverEnabled = true;

    void FixedUpdate()
    {
        if (!solverEnabled) return;
        if (publisher == null || fkSolver == null || targetController == null) return;

        ArticulationBody[] bodies = publisher.JointBodies;
        if (bodies == null) return;

        // Task-space error
        Vector3 posError = targetController.TargetPosition - fkSolver.GetEEFPosition();

        Quaternion errorQuat = targetController.TargetRotation * Quaternion.Inverse(fkSolver.GetEEFRotation());
        errorQuat.ToAngleAxis(out float errAngleRad, out Vector3 errAxis);
        // Wrap angle to [-PI, PI]
        if (errAngleRad > Mathf.PI) errAngleRad -= 2f * Mathf.PI;
        // Guard degenerate axis (zero rotation)
        if (errAxis.sqrMagnitude < 1e-6f) errAxis = Vector3.up;
        Vector3 oriError = errAxis.normalized * (errAngleRad * orientationWeight);

        if (posError.magnitude < positionTolerance &&
            Mathf.Abs(errAngleRad) * orientationWeight < orientationTolerance)
            return;

        // Geometric Jacobian (6x6)
        // For revolute joint i:
        //   translation row : cross(axis_i, p_EEF - p_i)
        //   rotation row    : axis_i  (weighted)
        //
        // Joint rotation axis in world space:
        //   bodies[i].transform.rotation * bodies[i].anchorRotation * Vector3.right
        //
        // Unity's URDF importer aligns the URDF joint axis with the
        // ArticulationBody's local X-axis (right) via anchorRotation.
        float[,] J = new float[6, 6];
        Vector3 eefPos = fkSolver.GetEEFPosition();

        for (int i = 0; i < 6; i++)
        {
            if (bodies[i] == null) continue;

            Vector3 axis = bodies[i].transform.rotation
                           * bodies[i].anchorRotation
                           * Vector3.right;

            Vector3 pivot = bodies[i].transform.position;
            Vector3 transCol = Vector3.Cross(axis, eefPos - pivot);

            J[0, i] = transCol.x;
            J[1, i] = transCol.y;
            J[2, i] = transCol.z;

            J[3, i] = axis.x * orientationWeight;
            J[4, i] = axis.y * orientationWeight;
            J[5, i] = axis.z * orientationWeight;
        }

        // Task vector dx (6x1)
        float[] dx = new float[6]
        {
            posError.x, posError.y, posError.z,
            oriError.x, oriError.y, oriError.z
        };

        // DLS: form (J^T J + lambda^2 * I) dTheta = J^T dx
        float lambda2 = dampingLambda * dampingLambda;

        // A = J^T J + lambda^2 * I  (6x6)
        float[,] A = new float[6, 6];
        for (int r = 0; r < 6; r++)
        {
            for (int c = 0; c < 6; c++)
            {
                float sum = 0f;
                for (int k = 0; k < 6; k++)
                    sum += J[k, r] * J[k, c];   // J^T[r,k] = J[k,r]
                A[r, c] = sum;
            }
            A[r, r] += lambda2;
        }

        // b = J^T dx  (6x1)
        float[] b = new float[6];
        for (int r = 0; r < 6; r++)
        {
            float sum = 0f;
            for (int k = 0; k < 6; k++)
                sum += J[k, r] * dx[k];
            b[r] = sum;
        }

        // Solve Ax = b by Gaussian elimination with partial pivoting
        float[] dTheta = GaussianSolve(A, b);
        if (dTheta == null) return;

        // Apply joint updates
        float[] current = publisher.GetCurrentJointAngles(); // degrees
        for (int i = 0; i < 6; i++)
        {
            float deltaDeg = Mathf.Clamp(dTheta[i] * Mathf.Rad2Deg, -maxStepDeg, maxStepDeg);
            publisher.SetJointAngleLocally(i, current[i] + deltaDeg);
        }
    }

    /// Solves a 6x6 linear system Ax = b in-place using Gaussian elimination
    /// with partial pivoting. Returns null if the matrix is singular.
    static float[] GaussianSolve(float[,] A, float[] b)
    {
        const int N = 6;

        // Build augmented matrix [A | b]
        float[,] M = new float[N, N + 1];
        for (int r = 0; r < N; r++)
        {
            for (int c = 0; c < N; c++) M[r, c] = A[r, c];
            M[r, N] = b[r];
        }

        // Forward elimination
        for (int col = 0; col < N; col++)
        {
            // Partial pivot: find row with largest absolute value in this column
            int pivotRow = col;
            float maxVal = Mathf.Abs(M[col, col]);
            for (int row = col + 1; row < N; row++)
            {
                float v = Mathf.Abs(M[row, col]);
                if (v > maxVal) { maxVal = v; pivotRow = row; }
            }

            if (maxVal < 1e-10f) return null; // effectively singular

            // Swap pivot row into position
            if (pivotRow != col)
            {
                for (int c = 0; c <= N; c++)
                {
                    float tmp = M[col, c];
                    M[col, c] = M[pivotRow, c];
                    M[pivotRow, c] = tmp;
                }
            }

            // Eliminate rows below
            float diagInv = 1f / M[col, col];
            for (int row = col + 1; row < N; row++)
            {
                float factor = M[row, col] * diagInv;
                for (int c = col; c <= N; c++)
                    M[row, c] -= factor * M[col, c];
            }
        }

        // Back substitution
        float[] x = new float[N];
        for (int row = N - 1; row >= 0; row--)
        {
            float sum = M[row, N];
            for (int c = row + 1; c < N; c++)
                sum -= M[row, c] * x[c];
            x[row] = sum / M[row, row];
        }
        return x;
    }
}
