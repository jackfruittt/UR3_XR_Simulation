// References:
// [1] Intel RealSense rs-pointcloud-stitching (C++), realsenseai/librealsense, Apache 2.0
//     https://github.com/realsenseai/librealsense/tree/master/wrappers/pointcloud/pointcloud-stitching
//     Core pattern: ProjectFramesOnOtherDevice() - per depth pixel:
//       1. rs2_deproject_pixel_to_point()   -> camera-space XYZ
//       2. rs2_transform_point_to_point()   -> world-space XYZ via extrinsics
//     Intel's "extrinsics" is a fixed inter-camera transform. Here it is the
//     time-varying camera world pose supplied by IMUSubscriber (IMU + AprilTag fusion).
//
// [2] Intel RealSense RsPointCloudRenderer.cs (Unity SDK), realsenseai/librealsense
//     https://github.com/realsenseai/librealsense/blob/master/wrappers/unity/Assets/RealSenseSDK2.0/Scripts/RsPointCloudRenderer.cs
//     Mesh pattern: MeshTopology.Points, IndexFormat.UInt32.

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

/// Accumulates RGB-D frames into a persistent world-space spatial map.
/// Core idea from Intel rs-pointcloud-stitching [1]:
///   Per pixel: deproject -> camera space -> world space via camera pose -> accumulate
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class PointCloudStitcher : MonoBehaviour
{
    [Header("References")]
    public ROSPointCloudRenderer liveRenderer;
    public Transform cameraTransform; // set by IMUSubscriber once pose is known

    [Header("Capture Settings")]
    public KeyCode captureKey = KeyCode.C;
    public float autoCaptureIntervalSeconds = 0f;
    public int maxAccumulatedPoints = 3_000_000;

    [Header("Visualisation")]
    public float pointSize = 2f;
    public Material stitchedMaterial;

    readonly List<Vector3> _worldPositions = new List<Vector3>();
    readonly List<Color32> _worldColours   = new List<Color32>();

    Mesh         _stitchedMesh;
    MeshFilter   _meshFilter;
    MeshRenderer _meshRenderer;
    bool         _meshDirty;

    void Start()
    {
        // TODO: init mesh (MeshTopology.Points, IndexFormat.UInt32, MarkDynamic)
        // TODO: auto-wire liveRenderer and cameraTransform if null
    }

    void Update()
    {
        // TODO: check captureKey, call CaptureFrame()
        // TODO: auto-capture timer, call CaptureFrame()
        // TODO: if _meshDirty, call RebuildMesh()
    }

    /// Reads the depth texture from liveRenderer, deprojects each pixel to
    /// camera-space XYZ (pinhole model), then multiplies by cameraTransform.localToWorldMatrix
    /// to get world-space positions. Appends to _worldPositions / _worldColours.
    /// Evicts oldest points when maxAccumulatedPoints is exceeded.
    /// Mirrors rs2_deproject_pixel_to_point + rs2_transform_point_to_point [1].
    public void CaptureFrame()
    {
        // TODO
    }

    /// Clears all accumulated points and the mesh.
    public void ClearMap()
    {
        // TODO
    }

    /// Uploads _worldPositions and _worldColours into _stitchedMesh.
    void RebuildMesh()
    {
        // TODO
    }

    void OnDestroy()
    {
        // TODO: release _stitchedMesh
    }
}
