// Author: Jackson Russell

using System;
using System.Collections.Generic;
using UnityEngine;

/// Two-stage Tsai-Lenz hand-eye calibration solver for the AX = XB problem.
///
/// For eye-in-hand configuration, A is the relative EEF motion and B is the
/// corresponding relative target motion as seen by the camera:
///
///   A_i = T_EEF_{i-1}^{-1} * T_EEF_i       (relative robot motion, from FK)
///   B_i = T_cam_{i-1}^{-1} * T_cam_i       (relative target motion, in camera frame)
///   X   = T_tool0_to_camera                 (the calibration result)
///
/// Stage 1: recover rotation R_X from the skew-symmetric constraint.
/// Stage 2: recover translation t_X by least-squares given R_X.
///
/// Reference:
///   Tsai, R.Y. and Lenz, R.K. (1989). A new technique for fully autonomous
///   and efficient 3D robotics hand/eye calibration. IEEE Transactions on
///   Robotics and Automation, 5(3), pp. 345-358.
public static class HandEyeSolver
{
    // Minimum number of motion pairs required to solve. Three gives an exactly
    // determined system; more pairs improve conditioning via least-squares.
    public const int MinPairs = 3;

    /// Solve for X given a list of (A, B) motion pairs.
    /// Returns the solved T_tool0_to_camera as a Unity Matrix4x4, or Matrix4x4.identity
    /// on failure (fewer than MinPairs valid pairs, or degenerate rotation).
    /// The out parameter residualDeg reports the mean rotation residual across all pairs
    /// as a sanity check; values below 2 degrees indicate a well-conditioned result.
    public static Matrix4x4 Solve(List<(Matrix4x4 A, Matrix4x4 B)> pairs, out float residualDeg)
    {
        residualDeg = float.NaN;

        if (pairs == null || pairs.Count < MinPairs)
        {
            Debug.LogWarning("[HandEyeSolver] Insufficient motion pairs; need at least " + MinPairs + ".");
            return Matrix4x4.identity;
        }

        // Stage 1: Rotation
        // For each pair construct the angle-axis form of R_A and R_B.
        // The Tsai-Lenz rotation equation is:
        //   skew(r_A + r_B) * r_X = r_B - r_A
        // where r = (theta / (2 * cos(theta/2))) * axis  (modified Rodrigues vector).
        // Stacking all pairs gives a 3N x 3 linear system solved in least-squares sense.

        int n = pairs.Count;
        float[,] M = new float[3 * n, 3];
        float[]  b = new float[3 * n];

        bool anySkipped = false;
        int  validRows  = 0;

        for (int i = 0; i < n; i++)
        {
            Quaternion qA = pairs[i].A.rotation;
            Quaternion qB = pairs[i].B.rotation;

            // Skip near-identity rotations (pure translations) to avoid ill-conditioning.
            float angleA = 2f * Mathf.Acos(Mathf.Clamp(Mathf.Abs(qA.w), 0f, 1f)) * Mathf.Rad2Deg;
            float angleB = 2f * Mathf.Acos(Mathf.Clamp(Mathf.Abs(qB.w), 0f, 1f)) * Mathf.Rad2Deg;
            if (angleA < 2f || angleB < 2f)
            {
                anySkipped = true;
                continue;
            }

            Vector3 rA = ModifiedRodrigues(qA);
            Vector3 rB = ModifiedRodrigues(qB);

            // Row block: skew(rA + rB)
            Vector3 s = rA + rB;
            int row = validRows * 3;
            M[row,     0] =  0f;   M[row,     1] = -s.z;  M[row,     2] =  s.y;
            M[row + 1, 0] =  s.z;  M[row + 1, 1] =  0f;   M[row + 1, 2] = -s.x;
            M[row + 2, 0] = -s.y;  M[row + 2, 1] =  s.x;  M[row + 2, 2] =  0f;

            Vector3 rhs = rB - rA;
            b[row]     = rhs.x;
            b[row + 1] = rhs.y;
            b[row + 2] = rhs.z;

            validRows++;
        }

        if (anySkipped)
            Debug.LogWarning("[HandEyeSolver] Some motion pairs skipped (rotation angle < 2 deg). Diversify waypoints.");

        if (validRows < MinPairs)
        {
            Debug.LogError("[HandEyeSolver] Too few valid rotation-rich pairs after filtering. Aborting.");
            return Matrix4x4.identity;
        }

        // Trim arrays to actual valid row count
        int rows = validRows * 3;
        float[,] Mtrim = new float[rows, 3];
        float[]  btrim = new float[rows];
        for (int r = 0; r < rows; r++)
        {
            Mtrim[r, 0] = M[r, 0]; Mtrim[r, 1] = M[r, 1]; Mtrim[r, 2] = M[r, 2];
            btrim[r]    = b[r];
        }

        // Solve M * r_X = b via SVD least-squares
        float[] rXArr = SolveLeastSquares3(Mtrim, btrim, rows);
        if (rXArr == null) return Matrix4x4.identity;

        Vector3    rX = new Vector3(rXArr[0], rXArr[1], rXArr[2]);
        Quaternion qX = InverseModifiedRodrigues(rX);
        Matrix3x3  RX = QuatToMatrix3x3(qX);

        // Stage 2: Translation
        // (R_A - I) * t_X = R_X * t_B - t_A
        // Stack all pairs into a 3N x 3 system and solve via least-squares.

        float[,] C = new float[rows, 3];
        float[]  d = new float[rows];
        validRows   = 0;

        for (int i = 0; i < n; i++)
        {
            Quaternion qA = pairs[i].A.rotation;
            float angleA  = 2f * Mathf.Acos(Mathf.Clamp(Mathf.Abs(qA.w), 0f, 1f)) * Mathf.Rad2Deg;
            if (angleA < 2f) continue;

            Matrix3x3 RA = QuatToMatrix3x3(pairs[i].A.rotation);
            Vector3   tA = ExtractTranslation(pairs[i].A);
            Vector3   tB = ExtractTranslation(pairs[i].B);

            // (R_A - I)
            float a00 = RA.m00 - 1f, a01 = RA.m01,      a02 = RA.m02;
            float a10 = RA.m10,      a11 = RA.m11 - 1f, a12 = RA.m12;
            float a20 = RA.m20,      a21 = RA.m21,      a22 = RA.m22 - 1f;

            Vector3 rhs2 = RX.MultiplyVector(tB) - tA;

            int row = validRows * 3;
            C[row,     0] = a00; C[row,     1] = a01; C[row,     2] = a02;
            C[row + 1, 0] = a10; C[row + 1, 1] = a11; C[row + 1, 2] = a12;
            C[row + 2, 0] = a20; C[row + 2, 1] = a21; C[row + 2, 2] = a22;
            d[row]     = rhs2.x;
            d[row + 1] = rhs2.y;
            d[row + 2] = rhs2.z;
            validRows++;
        }

        rows = validRows * 3;
        float[,] Ctrim = new float[rows, 3];
        float[]  dtrim = new float[rows];
        for (int r = 0; r < rows; r++)
        {
            Ctrim[r, 0] = C[r, 0]; Ctrim[r, 1] = C[r, 1]; Ctrim[r, 2] = C[r, 2];
            dtrim[r]    = d[r];
        }

        float[] tXArr = SolveLeastSquares3(Ctrim, dtrim, rows);
        if (tXArr == null) return Matrix4x4.identity;
        Vector3 tX = new Vector3(tXArr[0], tXArr[1], tXArr[2]);

        // Compute mean rotation residual across all valid pairs
        residualDeg = ComputeResidualDeg(pairs, qX, tX);

        return Matrix4x4.TRS(tX, qX, Vector3.one);
    }

    // Computes mean rotation residual: for each pair, ||R_A * R_X - R_X * R_B||_F in degrees.
    static float ComputeResidualDeg(List<(Matrix4x4 A, Matrix4x4 B)> pairs, Quaternion qX, Vector3 tX)
    {
        float total = 0f;
        int   count = 0;
        foreach (var (A, B) in pairs)
        {
            Quaternion qA      = A.rotation;
            float angleA       = 2f * Mathf.Acos(Mathf.Clamp(Mathf.Abs(qA.w), 0f, 1f)) * Mathf.Rad2Deg;
            if (angleA < 2f) continue;

            Quaternion lhs     = qA * qX;
            Quaternion rhs     = qX * B.rotation;
            float      dot     = Mathf.Abs(Quaternion.Dot(lhs, rhs));
            float      angleDeg = 2f * Mathf.Acos(Mathf.Clamp(dot, 0f, 1f)) * Mathf.Rad2Deg;
            total += angleDeg;
            count++;
        }
        return count > 0 ? total / count : float.NaN;
    }

    // Converts a quaternion to the modified Rodrigues parametrisation used by Tsai-Lenz.
    // r = (theta / (2 * cos(theta / 2))) * axis
    static Vector3 ModifiedRodrigues(Quaternion q)
    {
        // Ensure positive scalar part to avoid sign ambiguity
        if (q.w < 0f) { q.x = -q.x; q.y = -q.y; q.z = -q.z; q.w = -q.w; }

        float halfTheta = Mathf.Acos(Mathf.Clamp(q.w, 0f, 1f));
        float sinHalf   = Mathf.Sin(halfTheta);
        float theta     = 2f * halfTheta;
        float scale     = sinHalf > 1e-6f ? (theta / (2f * q.w)) : 1f;

        return new Vector3(q.x * scale, q.y * scale, q.z * scale);
    }

    // Inverts the modified Rodrigues parametrisation back to a unit quaternion.
    static Quaternion InverseModifiedRodrigues(Vector3 r)
    {
        float rMag = r.magnitude;
        float w    = 2f / Mathf.Sqrt(4f + rMag * rMag);
        float s    = w / 2f;
        return new Quaternion(r.x * s, r.y * s, r.z * s, w).normalized;
    }

    // Solves A * x = b (n x 3 overdetermined system) via thin SVD with 3 columns.
    // Returns the 3-element solution vector, or null if the system is singular.
    static float[] SolveLeastSquares3(float[,] A, float[] b, int n)
    {
        // Normal equations: (A^T A) x = A^T b
        // A^T A is 3x3; for a well-posed calibration this is sufficient.
        float[,] AtA = new float[3, 3];
        float[]  Atb = new float[3];

        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                Atb[j] += A[i, j] * b[i];
                for (int k = 0; k < 3; k++)
                    AtA[j, k] += A[i, j] * A[i, k];
            }
        }

        return Solve3x3(AtA, Atb);
    }

    // Solves the 3x3 linear system M * x = rhs using Cramer's rule.
    // Returns null if the determinant is below the stability threshold.
    static float[] Solve3x3(float[,] M, float[] rhs)
    {
        float det = M[0,0] * (M[1,1]*M[2,2] - M[1,2]*M[2,1])
                  - M[0,1] * (M[1,0]*M[2,2] - M[1,2]*M[2,0])
                  + M[0,2] * (M[1,0]*M[2,1] - M[1,1]*M[2,0]);

        if (Mathf.Abs(det) < 1e-8f)
        {
            Debug.LogError("[HandEyeSolver] 3x3 system is singular (det ~ 0). Motion pairs may be degenerate.");
            return null;
        }

        float invDet = 1f / det;

        float[,] adj = new float[3, 3]
        {
            {  (M[1,1]*M[2,2] - M[1,2]*M[2,1]) * invDet,
              -(M[0,1]*M[2,2] - M[0,2]*M[2,1]) * invDet,
               (M[0,1]*M[1,2] - M[0,2]*M[1,1]) * invDet },
            { -(M[1,0]*M[2,2] - M[1,2]*M[2,0]) * invDet,
               (M[0,0]*M[2,2] - M[0,2]*M[2,0]) * invDet,
              -(M[0,0]*M[1,2] - M[0,2]*M[1,0]) * invDet },
            {  (M[1,0]*M[2,1] - M[1,1]*M[2,0]) * invDet,
              -(M[0,0]*M[2,1] - M[0,1]*M[2,0]) * invDet,
               (M[0,0]*M[1,1] - M[0,1]*M[1,0]) * invDet }
        };

        return new float[]
        {
            adj[0,0]*rhs[0] + adj[0,1]*rhs[1] + adj[0,2]*rhs[2],
            adj[1,0]*rhs[0] + adj[1,1]*rhs[1] + adj[1,2]*rhs[2],
            adj[2,0]*rhs[0] + adj[2,1]*rhs[1] + adj[2,2]*rhs[2]
        };
    }

    static Vector3 ExtractTranslation(Matrix4x4 m)
        => new Vector3(m.m03, m.m13, m.m23);

    static Matrix3x3 QuatToMatrix3x3(Quaternion q)
    {
        float x = q.x, y = q.y, z = q.z, w = q.w;
        return new Matrix3x3(
            1f - 2f*(y*y + z*z),       2f*(x*y - z*w),       2f*(x*z + y*w),
                 2f*(x*y + z*w), 1f - 2f*(x*x + z*z),        2f*(y*z - x*w),
                 2f*(x*z - y*w),       2f*(y*z + x*w), 1f - 2f*(x*x + y*y)
        );
    }

    // Minimal 3x3 matrix for rotation operations (avoids Unity Matrix4x4 overhead).
    struct Matrix3x3
    {
        public float m00, m01, m02;
        public float m10, m11, m12;
        public float m20, m21, m22;

        public Matrix3x3(float a00, float a01, float a02,
                         float a10, float a11, float a12,
                         float a20, float a21, float a22)
        {
            m00=a00; m01=a01; m02=a02;
            m10=a10; m11=a11; m12=a12;
            m20=a20; m21=a21; m22=a22;
        }

        public Vector3 MultiplyVector(Vector3 v)
            => new Vector3(m00*v.x + m01*v.y + m02*v.z,
                           m10*v.x + m11*v.y + m12*v.z,
                           m20*v.x + m21*v.y + m22*v.z);
    }
}
