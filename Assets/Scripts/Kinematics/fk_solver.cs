using UnityEngine;

/// FK solver using Unity's Transform hierarchy.
/// Wraps tool0 and camera_link poses for IK and calibration consumption.

public class RobotFKSolver : MonoBehaviour
{
    [Header("Link Transforms (assign in Inspector)")]
    public Transform baseLinkTransform;
    public Transform shoulderLinkTransform;
    public Transform upperArmLinkTransform;
    public Transform forearmLinkTransform;
    public Transform wrist1LinkTransform;
    public Transform wrist2LinkTransform;
    public Transform wrist3LinkTransform;
    public Transform tool0Transform;

    public Transform tool0CameraTransform;

    public Matrix4x4 GetEEFMatrix()
    {
        // Returns the current T_tool0 in world space as a Matrix4x4
        return Util.TransformToMatrix(tool0Transform);
    }

    // Returns world-space pose of every link given current joint state
    public Matrix4x4[] GetAllLinkPoses()
    {
        return new Matrix4x4[]
        {
            Util.TransformToMatrix(baseLinkTransform),
            Util.TransformToMatrix(shoulderLinkTransform),
            Util.TransformToMatrix(upperArmLinkTransform),
            Util.TransformToMatrix(forearmLinkTransform),
            Util.TransformToMatrix(wrist1LinkTransform),
            Util.TransformToMatrix(wrist2LinkTransform),
            Util.TransformToMatrix(wrist3LinkTransform),
            Util.TransformToMatrix(tool0Transform),
            Util.TransformToMatrix(tool0CameraTransform),
        };
    }

    public Vector3 GetEEFPosition()  => tool0CameraTransform.position;
    /// Rotation from tool0 (gripper flange), not camera_link - avoids wrist contortion from camera offset.
    public Quaternion GetEEFRotation() => tool0Transform.rotation;
}
