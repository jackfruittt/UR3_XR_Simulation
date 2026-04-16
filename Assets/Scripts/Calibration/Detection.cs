// Author: Jackson Russell

using UnityEngine;
using System.Collections.Generic;
using AprilTag;

/// Main MonoBehaviour for AprilTag detection.
/// Reads the RGB texture from SimpleImageSubscriber each frame,
/// runs the TagDetector, then drives TagDrawer to show overlays.
public class Detection : MonoBehaviour
{
    [Header("Input")]
    // _colourSource is optional - Detection reads colour directly from _pointCloudRenderer.ColorTexture
    // to avoid a duplicate colour subscription and second GPU upload each frame.
    [SerializeField] SimpleImageSubscriber _colourSource;
    [SerializeField] ROSPointCloudRenderer _pointCloudRenderer;

    // IMU subscriber - provides live camera world position and orientation.
    // Used in GetDepthRefinedWorldPose so the tag-to-world conversion reflects
    // the physical camera pose from the Madgwick filter, not the static Unity
    // D455_Camera transform. Falls back to _camera.transform when IMU has no fix.
    [SerializeField] IMUSubscriber _imuSubscriber;

    // Colour texture sourced from the renderer (already double-buffered and uploaded).
    // Falls back to SimpleImageSubscriber if renderer is unavailable.
    Texture2D ColourTexture => _pointCloudRenderer != null
        ? _pointCloudRenderer.ColorTexture
        : _colourSource != null ? _colourSource.ColorTexture : null;

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

    // Intrinsics sourced from ROSPointCloudRenderer (updated live from camera_info by the renderer).
    // IMPORTANT: width/height must be the COLOUR image dimensions, not depth.
    // fx/fy/cx/cy come from /camera/camera/color/camera_info which is calibrated for the colour resolution.
    // Using depth dimensions here would cause ProjectToUV to divide by the wrong image size.
    public PoseEstimation.CameraIntrinsics Intrinsics => _pointCloudRenderer != null
        ? new PoseEstimation.CameraIntrinsics
          {
              fx     = _pointCloudRenderer.fx,
              fy     = _pointCloudRenderer.fy,
              cx     = _pointCloudRenderer.cx,
              cy     = _pointCloudRenderer.cy,
              width  = _pointCloudRenderer.ColorWidth,
              height = _pointCloudRenderer.ColorHeight
          }
        : default;

    [Header("Performance")]
    [Tooltip("Maximum AprilTag detection rate in Hz. Colour stream runs at 90 fps; " +
             "GetPixels32+Burst at full rate stalls the main thread every frame. " +
             "15 Hz is sufficient for calibration, 30 Hz for responsive overlays.")]
    [SerializeField] int _detectionHz = 15;

    // Tracks when the next detection should run (uses unscaledTime to stay stable during timescale changes)
    float _nextDetectionTime = 0f;

    bool _initialised = false;
    // Resolution the detector was last initialised at - if colorTexture is rebuilt at a
    // different size (e.g. first compressed frame arrives) we must reinitialise so the
    // TagDetector's internal ImageU8 buffer matches the incoming pixel count.
    // A mismatch causes Burst to read past the end of the managed Color32 array → null crash.
    int _detectorWidth  = 0;
    int _detectorHeight = 0;

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

        if (_imuSubscriber == null)
            _imuSubscriber = FindObjectOfType<IMUSubscriber>();

        // Create a default unlit magenta material if none assigned
        if (_tagMaterial == null)
        {
            _tagMaterial = new Material(Shader.Find("Unlit/Color"));
            _tagMaterial.color = new Color(1f, 0f, 1f);
        }

        if (_pointCloudRenderer == null) Debug.LogError("[Detection] Could not find ROSPointCloudRenderer.");
        if (_camera             == null) Debug.LogError("[Detection] Could not find D455_Camera Camera component.");
    }

    void Start()
    {
        _drawer = new TagDrawer(_tagMaterial);
        // Intrinsics are managed by ROSPointCloudRenderer which subscribes to camera_info
    }

    void LateUpdate()
    {
        if (ColourTexture == null) return;

        int tw = ColourTexture.width;
        int th = ColourTexture.height;

        // Reinitialise detector whenever the texture dimensions change.
        if (!_initialised || tw != _detectorWidth || th != _detectorHeight)
            InitDetector(tw, th);

        // Throttle detection - GetPixels32() + Burst readback is expensive; no benefit past ~15-30 Hz.
        // DrawTags always runs so overlays remain visible at full frame rate.
        if (Time.unscaledTime >= _nextDetectionTime)
        {
            _nextDetectionTime = Time.unscaledTime + 1f / Mathf.Max(1, _detectionHz);
            RunDetection();
        }

        DrawTags();
    }

    void OnDestroy()
    {
        _detector?.Dispose();
        _drawer?.Dispose();
    }

    /// Initialise (or reinitialise) the TagDetector at the given resolution.
    /// Also called automatically whenever colorTexture dimensions change.
    void InitDetector(int width, int height)
    {
        _detector?.Dispose(); // release old ImageU8 buffer before reallocating
        _detector       = new TagDetector(width, height, _decimation);
        _detectorWidth  = width;
        _detectorHeight = height;
        _initialised    = true;
        Debug.Log($"[Detection] Detector initialised at {width}x{height} (decimation={_decimation})");
    }

    /// Sample the RGB texture, run the detector, populate DetectedTags.
    void RunDetection()
    {
        // Guard: pixel count must exactly match what the detector's ImageU8 was allocated for.
        if (ColourTexture.width != _detectorWidth || ColourTexture.height != _detectorHeight)
            return;

        var pixels = ColourTexture.GetPixels32();
        if (pixels.Length != _detectorWidth * _detectorHeight)
            return; // texture returned a malformed array - skip this frame

        // TagDetector (PoseEstimationJob) uses: focalLength = height/2 / tan(fov/2)
        // This only produces the correct fy when fov = VERTICAL FOV IN RADIANS.
        float fovV;
        if (_pointCloudRenderer != null && _pointCloudRenderer.fy > 0)
            // Must use the COLOUR image height with the colour fy.
            // The detector was initialised at colour resolution; fovV must match that coordinate frame.
            fovV = 2f * Mathf.Atan(_pointCloudRenderer.ColorHeight / (2f * _pointCloudRenderer.fy));
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
    /// When the IMU has a valid pose, uses it for the camera-to-world conversion so the
    /// result reflects the physical camera orientation. Falls back to _camera.transform otherwise.
    /// Returns null if the tag is not currently detected or depth is unavailable.
    public Pose? GetDepthRefinedWorldPose(int tagId)
    {
        if (_detector == null) return null;

        foreach (var tag in _detector.DetectedTags)
        {
            if (tag.ID != tagId) continue;

            PoseEstimation.CameraIntrinsics intrinsics = Intrinsics;
            Pose cameraPose = PoseEstimation.GetDepthRefinedCameraPose(tag, DepthTexture, intrinsics);

            // Prefer the IMU world pose: it reflects the physical camera orientation
            // from the Madgwick filter and is valid whenever the filter has converged.
            if (_imuSubscriber != null && _imuSubscriber.PoseValid)
            {
                Vector3    imuPos = _imuSubscriber.CameraWorldPosition;
                Quaternion imuRot = _imuSubscriber.CameraWorldRotation;
                return PoseEstimation.CameraToWorldFromPose(cameraPose, imuPos, imuRot);
            }

            // Fallback: use the D455_Camera Unity Transform (correct for arm-mounted use
            // where the camera moves with FK, not relevant for freehand).
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
