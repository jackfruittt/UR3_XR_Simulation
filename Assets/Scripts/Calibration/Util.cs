using UnityEngine;


/// Coordinate frame and camera utilities shared across the calibration pipeline.
/// 
/// ROS optical frame  : X right, Y down, Z forward (into scene)
/// Unity camera space : X right, Y up,   Z forward (into scene)
/// -> only Y axis flips between the two
///
/// ROS uses left-hand -> right-hand quaternion (w, x, y, z vs x, y, z, w) -
static public class Util
{
    /// Returns the vertical field of view of a Unity Camera in radians.
    /// Use for TagDetector.ProcessImage — the library's PoseEstimationJob requires
    /// vertical FOV in radians: focalLength = height/2 / tan(fov/2).
    /// Unity's Camera.fieldOfView is always vertical, so just convert to radians.
    public static float VerticalFOVRadians(Camera camera)
    {
        return camera.fieldOfView * Mathf.Deg2Rad;
    }

    /// Returns the horizontal field of view of a Unity Camera in degrees.
    /// NOTE: Do NOT pass this to TagDetector.ProcessImage - it requires vertical FOV in radians.
    public static float HorizontalFOV(Camera camera)
    {
        return Camera.VerticalToHorizontalFieldOfView(camera.fieldOfView, camera.aspect);
    }

    /// Converts a position from ROS optical frame to Unity camera space.
    /// Needed if receiving pose data from ROS (e.g. static TF messages).
    public static Vector3 ROSOpticalToUnity(Vector3 rosPosition)
    {
        return new Vector3(rosPosition.x, -rosPosition.y, rosPosition.z);
    }

    /// Converts a Unity world-space Matrix4x4 to a ROS-convention 4x4 matrix.
    /// Used when publishing the solved T_tool0->camera_link back to ROS via ROS-TCP.
    /// Axis remapping:  ROS X = Unity Z  |  ROS Y = -Unity X  |  ROS Z = Unity Y
    public static Matrix4x4 UnityToROSMatrix(Matrix4x4 unityMatrix)
    {
        // Position
        // Decompose translation from column 3 of the Unity matrix
        Vector3 unityPos = new Vector3(unityMatrix.m03, unityMatrix.m13, unityMatrix.m23);

        // Apply axis remap to position
        Vector3 rosPos = new Vector3(
             unityPos.z,    // ROS X = Unity Z  (forward)
            -unityPos.x,    // ROS Y = -Unity X (left)
             unityPos.y     // ROS Z = Unity Y  (up)
        );

        // Rotation
        // Decompose rotation from the upper-left 3x3 of the Unity matrix
        Quaternion unityRot = unityMatrix.rotation;

        // Remap quaternion imaginary components using the same axis substitution.
        // The handedness flip introduces extra sign inversions on x and z.
        Quaternion rosRot = new Quaternion(
            -unityRot.z,    // ROS qx <-  Unity qz
             unityRot.x,    // ROS qy <-  Unity qx
            -unityRot.y,    // ROS qz <- -Unity qy
             unityRot.w     // scalar unchanged
        );

        return Matrix4x4.TRS(rosPos, rosRot, Vector3.one);
    }

    /// Converts a Unity Transform to a Matrix4x4 suitable for hand-eye calibration solver.
    /// Poses as 4x4 homogeneous transform matrices.
    public static Matrix4x4 TransformToMatrix(Transform t)
    {
        return Matrix4x4.TRS(t.position, t.rotation, Vector3.one);
    }

    /// Returns the current EEF (tool0) Matrix4x4 in world space.
    /// Pass the tool0 Transform from the robot hierarchy.
    public static Matrix4x4 GetEEFMatrix(Transform tool0)
    {
        // Only valid during a stationary capture (call after arm has settled)
        return TransformToMatrix(tool0);
    }
}
