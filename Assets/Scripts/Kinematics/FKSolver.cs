// Author: Jackson Russell

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

    // Pre-allocated - eliminates per-call heap allocation from GetAllLinkPoses().
    private readonly Matrix4x4[] _linkPoses = new Matrix4x4[9];

    public Matrix4x4 GetEEFMatrix()
    {
        // Returns the current T_tool0 in world space as a Matrix4x4
        return Util.TransformToMatrix(tool0Transform);
    }

    // Returns world-space pose of every link given current joint state.
    // The returned array is reused each call - read or copy values before the next call.
    public Matrix4x4[] GetAllLinkPoses()
    {
        _linkPoses[0] = Util.TransformToMatrix(baseLinkTransform);
        _linkPoses[1] = Util.TransformToMatrix(shoulderLinkTransform);
        _linkPoses[2] = Util.TransformToMatrix(upperArmLinkTransform);
        _linkPoses[3] = Util.TransformToMatrix(forearmLinkTransform);
        _linkPoses[4] = Util.TransformToMatrix(wrist1LinkTransform);
        _linkPoses[5] = Util.TransformToMatrix(wrist2LinkTransform);
        _linkPoses[6] = Util.TransformToMatrix(wrist3LinkTransform);
        _linkPoses[7] = Util.TransformToMatrix(tool0Transform);
        _linkPoses[8] = Util.TransformToMatrix(tool0CameraTransform);
        return _linkPoses;
    }

    public Vector3 GetEEFPosition()  => tool0CameraTransform.position;

    /// Rotation from tool0 (gripper flange), not camera_link - avoids wrist contortion from camera offset.
    public Quaternion GetEEFRotation() => tool0Transform.rotation;

    /// Returns T_EEF_camera: the pose of the camera frame expressed in tool0 (EEF) local space.
    /// This IS the hand-eye transform - read directly from the URDF hierarchy in simulation,
    /// eliminating the need for AX=XB calibration in the sim stage.
    /// Compare this against the AX=XB result on the real robot to validate your mount model.
    public Pose GetEEFCameraPose()
    {
        Vector3    localPos = tool0Transform.InverseTransformPoint(tool0CameraTransform.position);
        Quaternion localRot = Quaternion.Inverse(tool0Transform.rotation) * tool0CameraTransform.rotation;
        return new Pose(localPos, localRot);
    }

    /// Returns T_EEF_camera as a 4x4 matrix (robotics convention: [R|t; 0 0 0 1]).
    public Matrix4x4 GetEEFCameraMatrix()
    {
        Pose p = GetEEFCameraPose();
        return Matrix4x4.TRS(p.position, p.rotation, Vector3.one);
    }
}

