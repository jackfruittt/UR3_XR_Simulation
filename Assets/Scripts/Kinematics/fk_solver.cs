using UnityEngine;

/// A Forward kinematics solver levergaing Unity's built-in Transform hierarchy and Matrix4x4 class.
/// Used for computing the T_tool0->camera_link pose
/// 
/// The solver assumes a fixed, known offset between the robot's end-effector (tool0) and the camera_link.
/// The solver computes the T_tool0->camera_link pose by applying the known offset to the current T_tool0 pose obtained 
/// from the robot's Transform in Unity.

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
    public Quaternion GetEEFRotation() => tool0CameraTransform.rotation;
}
