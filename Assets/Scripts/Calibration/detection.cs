using UnityEngine;
using System.Collections.Generic;
using AprilTag;

/// Main MonoBehaviour for AprilTag detection.
/// Reads the RGB texture from SimpleImageSubscriber each frame,
/// runs the TagDetector, then drives TagDrawer to show overlays.
public class Detection : MonoBehaviour
{
    [Header("Input")]
    [SerializeField] SimpleImageSubscriber _colourSource;
    // Reference ROSPointCloudRenderer for depth - reuses its existing depthTexture (RFloat)
    // rather than adding a second aligned depth subscriber. Leave _depthSource empty if using this.
    [SerializeField] ROSPointCloudRenderer _pointCloudRenderer;

    // Helper to get whichever depth texture is available
    Texture2D DepthTexture => _pointCloudRenderer != null
        ? _pointCloudRenderer.DepthTexture
        : null;

    [Header("Detector Settings")]
    // Physical size of tag in metres
    [SerializeField] float _tagSize = 0.0556f;
    // Downscale factor before detection - higher, faster but less accurate. 1 = full res, 2 = half res, etc.
    [SerializeField] int _decimation = 2;

    [Header("Visualisation")]
    [SerializeField] Material _tagMaterial;

    // The Unity Camera component on D455_Camera
    // Horizontal FOV is passed to ProcessImage so depth is correctly estimated.
    [SerializeField] Camera _camera;

    // Native detector - must be Disposed on destroy
    TagDetector _detector;

    TagDrawer _drawer;

    // Latest detections - read by PoseEstimation and future hand-eye calibration collector
    public IEnumerable<AprilTag.TagPose> DetectedTags => _detector?.DetectedTags;

    // Count of tags seen in the most recent frame - read by CalibrationHUD
    public int LastTagCount { get; private set; }

    // Camera transform used for world-space conversions - read by CalibrationHUD corner projection
    public Transform CameraTransform => _camera != null ? _camera.transform : null;

    // Physical tag size in metres - read by CalibrationHUD for corner projection
    public float TagSize => _tagSize;

    // Intrinsics sourced from ROSPointCloudRenderer (updated live from camera_info by the renderer)
    // Exposed for PoseEstimation.ProjectToUV callers
    public PoseEstimation.CameraIntrinsics Intrinsics => _pointCloudRenderer != null
        ? new PoseEstimation.CameraIntrinsics
          {
              fx     = _pointCloudRenderer.fx,
              fy     = _pointCloudRenderer.fy,
              cx     = _pointCloudRenderer.cx,
              cy     = _pointCloudRenderer.cy,
              width  = _pointCloudRenderer.width,
              height = _pointCloudRenderer.height
          }
        : default;

    bool _initialised = false;

    void Awake()
    {
        // Auto-wire any Inspector fields left unassigned
        if (_colourSource == null)
            _colourSource = FindObjectOfType<SimpleImageSubscriber>();

        if (_pointCloudRenderer == null)
            _pointCloudRenderer = FindObjectOfType<ROSPointCloudRenderer>();

        if (_camera == null)
        {
            var camGO = GameObject.Find("D455_Camera");
            if (camGO != null) _camera = camGO.GetComponent<Camera>();
        }

        // Create a default unlit magenta material if none assigned
        if (_tagMaterial == null)
        {
            _tagMaterial = new Material(Shader.Find("Unlit/Color"));
            _tagMaterial.color = new Color(1f, 0f, 1f);
        }

        if (_colourSource == null) Debug.LogError("[Detection] Could not find SimpleImageSubscriber.");
        if (_camera       == null) Debug.LogError("[Detection] Could not find D455_Camera Camera component.");
    }

    void Start()
    {
        _drawer = new TagDrawer(_tagMaterial);
        // Intrinsics are managed by ROSPointCloudRenderer which subscribes to camera_info
    }

    void LateUpdate()
    {
        if (_colourSource.ColorTexture == null) return;

        if (!_initialised)
            InitDetector(_colourSource.ColorTexture.width, _colourSource.ColorTexture.height);

        RunDetection();
        DrawTags();
    }

    void OnDestroy()
    {
        _detector?.Dispose();
        _drawer?.Dispose();
    }

    /// Initialise the TagDetector once we know the input resolution.
    /// Call once from LateUpdate when _colourSource.ColorTexture is first non-null.
    void InitDetector(int width, int height)
    {
        _detector = new TagDetector(width, height, _decimation);
        _initialised = true;
    }

    /// Sample the RGB texture, run the detector, populate DetectedTags.
    void RunDetection()
    {
        var pixels = _colourSource.ColorTexture.GetPixels32();

        // TagDetector (PoseEstimationJob) uses: focalLength = height/2 / tan(fov/2)
        // This only produces the correct fy when fov = VERTICAL FOV IN RADIANS.
        // Using horizontal FOV or degrees gives a completely wrong focal length.
        float fovV;
        if (_pointCloudRenderer != null && _pointCloudRenderer.fy > 0)
            fovV = 2f * Mathf.Atan(_pointCloudRenderer.height / (2f * _pointCloudRenderer.fy));
        else
            // Note: Unity Camera.fieldOfView is always vertical (Unity docs), perform radians conversion here only
            fovV = _camera != null ? _camera.fieldOfView * Mathf.Deg2Rad : 1.0f;

        _detector.ProcessImage(pixels, fovV, _tagSize);
    }

    /// Tell TagDrawer to render an overlay for each detected tag this frame.
    void DrawTags()
    {
        var detected = new System.Collections.Generic.List<int>();
        foreach (var tag in _detector.DetectedTags)
        {
            // Uncomment to enable 3D quad overlay
            // Vector3 worldPos = _camera.transform.TransformPoint(tag.Position);
            // Quaternion worldRot = _camera.transform.rotation * tag.Rotation;
            // _drawer.Draw(tag.ID, worldPos, worldRot, _tagSize);
            detected.Add(tag.ID);
        }
        _drawer.HideUndetected(detected);
        LastTagCount = detected.Count;
    }

    /// Returns the depth-refined world-space pose of a detected tag by ID.
    /// Combines AprilTag detection (RGB) with aligned depth sampling (D) - Pass deliverable.
    /// Returns null if the tag is not currently detected or depth is unavailable.
    public Pose? GetDepthRefinedWorldPose(int tagId)
    {
        if (_detector == null) return null;

        foreach (var tag in _detector.DetectedTags)
        {
            if (tag.ID != tagId) continue;

            PoseEstimation.CameraIntrinsics intrinsics = Intrinsics;
            Pose cameraPose = PoseEstimation.GetDepthRefinedCameraPose(tag, DepthTexture, intrinsics);
            return PoseEstimation.CameraToWorld(cameraPose, _camera.transform);
        }

        return null;
    }

    /// Returns the depth-refined world-space pose as a full 4x4 SE(3) matrix.
    /// Top-left 3x3 = rotation matrix R; right column = translation t.
    /// Returns null if the tag is not detected or depth is unavailable.
    public Matrix4x4? GetDepthRefinedWorldMatrix(int tagId)
    {
        Pose? pose = GetDepthRefinedWorldPose(tagId);
        if (!pose.HasValue) return null;
        return PoseEstimation.PoseToMatrix(pose.Value);
    }
}
