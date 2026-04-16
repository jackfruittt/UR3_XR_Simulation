// Author: Jackson Russell

using UnityEngine;

/// Singularity detection for the UR3e.
///
/// Reference:
///  Villalobos et al. (2022): https://www.mdpi.com/2218-6581/11/6/137
///
/// Notes:
///  det(J) = sin(q3) * sin(q5) * (a2*c2 + a3*c23 + d5*s234)
///  Elbow:    sin(q3) = 0  (arm fully stretched or folded)
///  Wrist:    sin(q5) = 0  (J4 and J6 axes become parallel, lose 1 rotational DOF)
///  Shoulder: a2*c2 + a3*c23 + d5*s234 = 0  (wrist centre lies on base axis)

public static class SingularityChecker
{
    // UR3e Modified DH Parameters (Villalobos convention: alpha_{i-1}, a_{i-1}, d_i)
    // Lengths in metres. Theta offsets (+90deg on J2, -90deg on J4) applied at solve time.
    const float alpha1 = 0f;               const float a1 = 0f;      const float d1 = 0.152f;  // J1
    const float alpha2 = Mathf.PI / 2f;    const float a2 = 0.244f;  const float d2 = 0f;      // J2
    const float alpha3 = 0f;               const float a3 = 0.213f;  const float d3 = 0f;      // J3
    const float alpha4 = 0f;               const float a4 = 0f;      const float d4 = 0.131f;  // J4
    const float alpha5 = -Mathf.PI / 2f;   const float a5 = 0f;      const float d5 = 0.085f;  // J5
    const float alpha6 = Mathf.PI / 2f;    const float a6 = 0f;      const float d6 = 0.092f;  // J6

    const float Epsilon = 1e-5f;   // float-safe zero threshold (paper uses 1e-12)

    /// Combined result returned by Evaluate() — avoids recomputing trig when
    /// both the determinant and singularity type are needed in the same frame.
    public struct EvalResult
    {
        public float           Determinant;   // det(J) = sin(q3)*sin(q5)*shoulderTerm
        public SingularityType Type;
        public bool            IsElbow;
        public bool            IsWrist;
        public bool            IsShoulder;
    }

    // Combined classification - returns active type(s), or None.
    public enum SingularityType { None, Elbow, Wrist, Shoulder, Multiple }

    /// Single-pass evaluation: computes all trig values exactly once, returns det(J) and type.
    /// q zero-indexed: q[0]=J1 .. q[5]=J6, radians.
    public static EvalResult Evaluate(float[] q)
    {
        float s3           = Mathf.Sin(q[2]);
        float s5           = Mathf.Sin(q[4]);
        float c2           = Mathf.Cos(q[1]);
        float c23          = Mathf.Cos(q[1] + q[2]);
        float s234         = Mathf.Sin(q[1] + q[2] + q[3]);
        float shoulderTerm = a2 * c2 + a3 * c23 + d5 * s234;
        float det          = s3 * s5 * shoulderTerm;

        bool elbow    = Mathf.Abs(s3)           < Epsilon;
        bool wrist    = Mathf.Abs(s5)           < Epsilon;
        bool shoulder = Mathf.Abs(shoulderTerm) < Epsilon;

        int count = (elbow ? 1 : 0) + (wrist ? 1 : 0) + (shoulder ? 1 : 0);

        SingularityType type;
        if      (count == 0) type = SingularityType.None;
        else if (count >  1) type = SingularityType.Multiple;
        else if (elbow)      type = SingularityType.Elbow;
        else if (wrist)      type = SingularityType.Wrist;
        else                 type = SingularityType.Shoulder;

        return new EvalResult
        {
            Determinant = det,
            Type        = type,
            IsElbow     = elbow,
            IsWrist     = wrist,
            IsShoulder  = shoulder,
        };
    }

    // det(J) = sin(q3)*sin(q5)*(a2*c2 + a3*c23 + d5*s234)  (Eq. 46, Villalobos)
    // q zero-indexed: q[0]=J1 .. q[5]=J6, radians.
    public static float JacobianDeterminant(float[] q)
    {
        float s3   = Mathf.Sin(q[2]);
        float s5   = Mathf.Sin(q[4]);
        float c2   = Mathf.Cos(q[1]);
        float c23  = Mathf.Cos(q[1] + q[2]);
        float s234 = Mathf.Sin(q[1] + q[2] + q[3]);
        float shoulderTerm = a2 * c2 + a3 * c23 + d5 * s234;
        return s3 * s5 * shoulderTerm;
    }

    // Elbow: sin(q3) ~= 0
    public static bool IsElbowSingular(float[] q)
    {
        return Mathf.Abs(Mathf.Sin(q[2])) < Epsilon;
    }

    // Wrist: sin(q5) ~= 0
    public static bool IsWristSingular(float[] q)
    {
        return Mathf.Abs(Mathf.Sin(q[4])) < Epsilon;
    }

    // Shoulder: a2*c2 + a3*c23 + d5*s234 ~= 0
    public static bool IsShoulderSingular(float[] q)
    {
        float c2   = Mathf.Cos(q[1]);
        float c23  = Mathf.Cos(q[1] + q[2]);
        float s234 = Mathf.Sin(q[1] + q[2] + q[3]);
        float shoulderTerm = a2 * c2 + a3 * c23 + d5 * s234;
        return Mathf.Abs(shoulderTerm) < Epsilon;
    }

    // Combined classification - returns active type(s), or None.
    // Wrap Evaluate() internally to avoid redundant trig computation.
    public static SingularityType Classify(float[] q) => Evaluate(q).Type;
}

