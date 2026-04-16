// Author: Jackson Russell

using UnityEngine;

/// Utilities for extracting a tag's 3D pose from the detector output and depth texture.
/// Static helper class - not a MonoBehaviour.
/// Used by Detection.cs (object pose in camera space) and
/// the hand-eye calibration collector (EEF + tag pose pairs).
public static class PoseEstimation
{
    // Camera intrinsics - sourced from ROSPointCloudRenderer (updated from camera_info)

    /// D455 pinhole camera intrinsics.
    /// ROSPointCloudRenderer subscribes to /camera_info and keeps these current.
    /// Detection.cs builds this struct on demand from the renderer's public fields.
    public struct CameraIntrinsics
    {
        public float fx;     // focal length x (pixels)
        public float fy;     // focal length y (pixels)
        public float cx;     // principal point x (pixels)
        public float cy;     // principal point y (pixels)
        public int   width;  // image width  (pixels)
        public int   height; // image height (pixels)

        public bool IsValid => fx > 0 && fy > 0;

        /// Horizontal FOV in degrees derived from fx.
        /// More accurate than using the Unity Camera field of view.
        public float HorizontalFOV()
        {
            return 2f * Mathf.Atan(width / (2f * fx)) * Mathf.Rad2Deg;
        }
    }

    /// Returns the tag pose in camera-local space directly from the detector.
    /// Monocular estimate: position from tag size + FOV, no depth texture needed.
    /// Position is in metres, Unity camera space (Z = forward into scene).
    public static Pose GetCameraPose(AprilTag.TagPose tag)
    {
        return new Pose(tag.Position, tag.Rotation);
    }

    /// Full RGBD pose: takes the monocular camera-space pose from the detector and
    /// replaces the Z component with a measurement from the aligned depth texture.
    public static Pose GetDepthRefinedCameraPose(
        AprilTag.TagPose tag,
        Texture2D depthTexture,
        CameraIntrinsics intrinsics)
    {
        Pose monocularPose = GetCameraPose(tag);

        if (depthTexture == null || !intrinsics.IsValid)
            return monocularPose;

        // Project the detector's camera-space position onto the image plane
        // Use pinhole intrinsics to find the correct depth pixel
        Vector2 uv    = ProjectToUV(monocularPose.position, intrinsics);
        float   depth = SampleDepth(uv, depthTexture);

        if (depth < 0f)
            return monocularPose; // depth invalid - fall back to monocular

        // Replace Z with measured depth; scale X and Y to match
        // Camera-space: X = (u - cx) / fx * Z,  Y = (v - cy) / fy * Z
        float u_px = uv.x * intrinsics.width;
        float v_px = uv.y * intrinsics.height;
        Vector3 refinedPos = new Vector3(
            (u_px - intrinsics.cx) / intrinsics.fx * depth,
            (v_px - intrinsics.cy) / intrinsics.fy * depth,
            depth
        );

        return new Pose(refinedPos, monocularPose.rotation);
    }

    /// Converts a camera-space pose to world space.
    /// cameraTransform: the Transform of the D455_Camera GameObject.
    public static Pose CameraToWorld(Pose cameraPose, Transform cameraTransform)
    {
        Vector3    worldPos = cameraTransform.TransformPoint(cameraPose.position);
        Quaternion worldRot = cameraTransform.rotation * cameraPose.rotation;
        return new Pose(worldPos, worldRot);
    }

    /// Converts a camera-space pose to world space using an explicit position and rotation.
    /// Use this when the camera world pose comes from the IMU rather than a Unity Transform,
    /// since the IMU filter runs independently of the scene hierarchy.
    /// Equivalent to Matrix4x4.TRS(camWorldPos, camWorldRot, one).MultiplyPoint(cameraPose.position).
    public static Pose CameraToWorldFromPose(Pose cameraPose, Vector3 camWorldPos, Quaternion camWorldRot)
    {
        Vector3    worldPos = camWorldPos + camWorldRot * cameraPose.position;
        Quaternion worldRot = camWorldRot * cameraPose.rotation;
        return new Pose(worldPos, worldRot);
    }

    /// Extracts T_EEF_camera directly from the URDF scene hierarchy.
    /// eefTransform:    tool0 Transform (EEF flange).
    /// cameraTransform: camera optical frame Transform (child of the kinematic chain).
    public static Pose CameraInEEFFrame(Transform eefTransform, Transform cameraTransform)
    {
        Vector3    localPos = eefTransform.InverseTransformPoint(cameraTransform.position);
        Quaternion localRot = Quaternion.Inverse(eefTransform.rotation) * cameraTransform.rotation;
        return new Pose(localPos, localRot);
    }

    /// Converts a world-space pose into the robot base frame.
    /// robotBase: the Transform of the UR3e base link (world child whose local space IS the base frame).
    /// Uses FK-derived camera position
    public static Pose WorldToBase(Pose worldPose, Transform robotBase)
    {
        Vector3    basePos = robotBase.InverseTransformPoint(worldPose.position);
        Quaternion baseRot = Quaternion.Inverse(robotBase.rotation) * worldPose.rotation;
        return new Pose(basePos, baseRot);
    }

    /// Returns a 4x4 homogeneous transformation matrix for a pose (SE(3)).
    /// Layout: top-left 3x3 is the rotation matrix R, right column is translation t.
    /// Compatible with standard robotics convention: [R | t; 0 0 0 1].
    /// Scale is always (1,1,1) - pure rigid body transformation.
    public static Matrix4x4 PoseToMatrix(Pose pose)
    {
        return Matrix4x4.TRS(pose.position, pose.rotation, Vector3.one);
    }

    // Depth sampling

    /// Samples depth from ROSPointCloudRenderer's depthTexture (TextureFormat.R16).
    /// R16 is unsigned normalised: GetPixel().r returns [0,1]; multiply by 65535 to recover raw mm.
    /// Uses a median over a 9x9 patch to reject noise.
    /// Returns depth in metres, or -1 if fewer than 4 valid pixels found in the patch.
    public static float SampleDepth(Vector2 imageUV, Texture2D depthTexture)
    {
        const int   halfPatch  = 4;              // 9x9 patch = 81 pixels
        const int   minValid   = 4;              // require at least 4 valid readings
        const float minDepthMM = 200f;           // discard below 20 cm (D455 min range margin)
        const float maxDepthMM = 6000f;          // discard above 6 m

        int cx = Mathf.Clamp((int)(imageUV.x * depthTexture.width),  0, depthTexture.width  - 1);
        int cy = Mathf.Clamp((int)(imageUV.y * depthTexture.height), 0, depthTexture.height - 1);

        // Collect valid depth readings from the patch into a small fixed-size buffer
        float[] samples = new float[81];
        int count = 0;

        int x0 = Mathf.Max(cx - halfPatch, 0);
        int x1 = Mathf.Min(cx + halfPatch, depthTexture.width  - 1);
        int y0 = Mathf.Max(cy - halfPatch, 0);
        int y1 = Mathf.Min(cy + halfPatch, depthTexture.height - 1);

        for (int py = y0; py <= y1; py++)
        {
            for (int px = x0; px <= x1; px++)
            {
                // R16 normalised [0,1] -> multiply by 65535 to recover raw uint16 mm value.
                float d = depthTexture.GetPixel(px, py).r * 65535f;
                if (d >= minDepthMM && d <= maxDepthMM)
                    samples[count++] = d;
            }
        }

        if (count < minValid) return -1f;

        // Partial insertion sort to find the median - O(n) for small patches.
        int medianIdx = count / 2;
        for (int i = 0; i <= medianIdx; i++)
        {
            int minJ = i;
            for (int j = i + 1; j < count; j++)
                if (samples[j] < samples[minJ]) minJ = j;
            float tmp = samples[i]; samples[i] = samples[minJ]; samples[minJ] = tmp;
        }

        return samples[medianIdx] / 1000f; // mm -> metres
    }

    /// Samples depth from a SimpleImageSubscriber depth texture (TextureFormat.RGBA32, 16UC1 packed).
    /// R = low byte, G = high byte of the 16-bit depth value in millimetres.
    public static float SampleDepthPacked16(Vector2 imageUV, Texture2D depthTexture)
    {
        int x = Mathf.Clamp((int)(imageUV.x * depthTexture.width),  0, depthTexture.width  - 1);
        int y = Mathf.Clamp((int)(imageUV.y * depthTexture.height), 0, depthTexture.height - 1);

        Color32 px  = depthTexture.GetPixel(x, y);
        ushort  raw = (ushort)(px.r | (px.g << 8));

        if (raw == 0) return -1f;

        return raw / 1000f;
    }

    // Projection

    /// Returns the normalised UV coordinate [0,1] of a camera-space point projected
    /// onto the image plane using the actual D455 pinhole intrinsics.
    /// More accurate than Unity's Camera.WorldToViewportPoint - uses real cx/cy.
    /// cameraSpacePosition.z must be > 0 (point must be in front of the camera).
    public static Vector2 ProjectToUV(Vector3 cameraSpacePosition, CameraIntrinsics intrinsics)
    {
        if (cameraSpacePosition.z <= 0f)
            return Vector2.zero;

        // Pinhole projection into image space (Y increases downward).
        // tag.Position is in Unity camera space (Y-up), so Y must be negated before projecting.
        // The library's PoseEstimationJob applies: pos.y *= -1 to convert from image Y-down to Unity Y-up.
        // Reverse that here: image_y = -unity_y
        float u = ( cameraSpacePosition.x / cameraSpacePosition.z) * intrinsics.fx + intrinsics.cx;
        float v = (-cameraSpacePosition.y / cameraSpacePosition.z) * intrinsics.fy + intrinsics.cy;

        return new Vector2(u / intrinsics.width, v / intrinsics.height);
    }
}
