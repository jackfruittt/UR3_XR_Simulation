// Author: Jackson Russell

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.InputSystem;
using TMPro;

/// Displays the D455 colour feed in a corner overlay, draws 2D tag corner markers
/// projected onto the feed image, and shows per-tag pose estimation results.
/// Calibration session controls are in HUDController (UI Toolkit).
///
/// Wiring (Inspector):
///   _colourSource        -> ImageSubscriber (SimpleImageSubscriber)
///   _detection           -> CalibrationManager (Detection)
///   _feedImage           -> CameraFeed (RawImage)
///   _statusText          -> TagInfoText (TMP)
///   _poseText            -> PoseText (TMP)
///   _hudPanel            -> CameraHUD (RectTransform)
///
/// Key bindings:
///   F9  - toggle HUD visibility
public class CalibrationHUD : MonoBehaviour
{
    [Header("Data Sources")]
    [SerializeField] SimpleImageSubscriber _colourSource;
    [SerializeField] Detection _detection;

    [Header("UI Elements")]
    [SerializeField] RawImage _feedImage;
    [SerializeField] TextMeshProUGUI _statusText;   // bottom strip: tag count
    [SerializeField] TextMeshProUGUI _poseText;     // middle strip: per-tag pose

    [Header("Corner Markers")]
    [Tooltip("Size of each corner dot in pixels")]
    [SerializeField] float _markerSize = 6f;
    [Tooltip("Marker colour (set alpha < 1 for semi-transparent)")]
    [SerializeField] Color _markerColour = new Color(0f, 1f, 0.2f, 0.9f);

    [Header("Axis Frame")]
    [Tooltip("Length of each axis line as a fraction of the tag size")]
    [SerializeField] float _axisLengthFraction = 0.6f;
    [Tooltip("Pixel width of drawn axis lines")]
    [SerializeField] float _axisLineWidth = 2.5f;

    [Header("Robot Base Frame")]
    [Tooltip("Transform of the UR3e base_link. Used to express tag position in the robot base frame " +
             "via FK (no hand-eye calibration required, provided the virtual mount matches physical).")]
    [SerializeField] Transform _robotBase;

    [Header("Hand-Eye (Simulated)")]
    [Tooltip("RobotFKSolver supplies tool0Transform + tool0CameraTransform. " +
             "T_EEF_camera is read from the URDF hierarchy at startup and logged to the console. " +
             "Use this value as your simulated hand-eye calibration result.")]
    [SerializeField] RobotFKSolver _fkSolver;
    [Header("IMU Diagnostics")]
    [Tooltip("IMUSubscriber to read diagnostics from. Auto-wired if left empty.")]
    [SerializeField] IMUSubscriber _imuSubscriber;
    [Tooltip("TextMeshPro element that shows IMU diagnostics. Create a new TMP text named \"IMUDiagText\" under the HUD panel.")]
    [SerializeField] TextMeshProUGUI _imuText;
    // Cached simulated T_EEF_camera (tool0 -> camera_link, local space).
    // Logged at startup; shown in status strip when no tags are visible.
    Pose _tEEFCamera;

    [Header("Panel Visibility")]
    [SerializeField] bool _visible = true;
    // Toggle the HUD panel with F9 (hardcoded to avoid Key enum serialisation mismatch)
    [SerializeField] RectTransform _hudPanel;

    [Header("Layout")]
    [Tooltip("If enabled, grows the PoseText and panel height at runtime so extra lines (cam/world/base) are not clipped.")]
    [SerializeField] bool _autoResizeHud = true;

    float _basePoseTextHeight;
    float _baseHudPanelHeight;
    float _baseImuTextHeight;

    // Pooled corner marker RectTransforms; expanded on demand.
    // Each detected tag uses 4 consecutive entries.
    readonly List<RectTransform> _markerPool = new List<RectTransform>();

    // Pooled axis line RectTransforms - 3 per tag (X=red, Y=cyan, Z=blue).
    // Each entry is a thin elongated Image rotated between projected start/end points.
    readonly List<RectTransform> _axisLinePool = new List<RectTransform>();
    static readonly Color[] AxisColors = {
        new Color(1f,   0.2f, 0.2f, 1f),   // X - red
        new Color(0.2f, 1f,   1f,   1f),   // Y - cyan
        new Color(0.3f, 0.5f, 1f,   1f),   // Z - blue
    };

    void Awake()
    {
        // Auto-wire any Inspector fields left unassigned
        if (_colourSource == null)
            _colourSource = FindObjectOfType<SimpleImageSubscriber>();

        if (_detection == null)
            _detection = FindObjectOfType<Detection>();

        if (_feedImage == null)
        {
            var go = GameObject.Find("CameraFeed");
            if (go != null) _feedImage = go.GetComponent<UnityEngine.UI.RawImage>();
        }

        if (_statusText == null)
        {
            var go = GameObject.Find("TagInfoText");
            if (go != null) _statusText = go.GetComponent<TMPro.TextMeshProUGUI>();
        }

        if (_poseText == null)
        {
            var go = GameObject.Find("PoseText");
            if (go != null) _poseText = go.GetComponent<TMPro.TextMeshProUGUI>();
        }

        if (_hudPanel == null)
        {
            var go = GameObject.Find("CameraHUD");
            if (go != null) _hudPanel = go.GetComponent<RectTransform>();
        }

        // If PoseText still doesn't exist, create it at runtime so tag pose data is always visible.
        if (_poseText == null)
        {
            Transform parent = _hudPanel != null ? (Transform)_hudPanel
                             : _statusText != null ? _statusText.transform.parent
                             : FindObjectOfType<Canvas>() != null ? FindObjectOfType<Canvas>().transform
                             : transform;
            var go2 = new GameObject("PoseText");
            go2.transform.SetParent(parent, false);
            _poseText = go2.AddComponent<TMPro.TextMeshProUGUI>();
            _poseText.fontSize           = 16;
            _poseText.color              = Color.white;
            _poseText.richText           = true;
            _poseText.enableWordWrapping = false;
            _poseText.overflowMode       = TMPro.TextOverflowModes.Overflow;
            var rt = _poseText.rectTransform;
            rt.anchorMin = new Vector2(0f, 1f);
            rt.anchorMax = new Vector2(1f, 1f);
            rt.pivot     = new Vector2(0f, 1f);
            // Position it just below the top of the panel, above the camera feed
            rt.offsetMin = new Vector2(8f, -300f);
            rt.offsetMax = new Vector2(-8f, -8f);
            Debug.Log("[CalibrationHUD] Created runtime PoseText element (assign PoseText in Inspector to avoid this).");
        }

        if (_feedImage  == null) Debug.LogWarning("[CalibrationHUD] CameraFeed RawImage not found.");
        if (_statusText == null) Debug.LogWarning("[CalibrationHUD] TagInfoText TMP not found.");

        // Auto-wire robot base_link (used for FK-based tag-in-base-frame display)
        if (_robotBase == null)
        {
            var go = GameObject.Find("base_link");
            if (go != null) _robotBase = go.transform;
            if (_robotBase == null) Debug.LogWarning("[CalibrationHUD] Robot base_link not found – assign _robotBase in Inspector for base-frame tag pose.");
        }

        // Auto-wire FK solver and extract simulated T_EEF_camera from the URDF hierarchy.
        if (_fkSolver == null)
            _fkSolver = FindObjectOfType<RobotFKSolver>();

        // Auto-wire IMU subscriber and diagnostics text.
        if (_imuSubscriber == null)
            _imuSubscriber = FindObjectOfType<IMUSubscriber>();
        if (_imuText == null)
        {
            // IMU diagnostics share the PoseText element in this scene.
            var go = GameObject.Find("PoseText");
            if (go != null) _imuText = go.GetComponent<TMPro.TextMeshProUGUI>();
        }
        // Allow text to grow past its authored rect without clipping.
        if (_imuText != null)
        {
            _imuText.enableWordWrapping = false;
            _imuText.overflowMode      = TMPro.TextOverflowModes.Overflow;
            _imuText.color             = Color.white;
        }
        if (_statusText != null) _statusText.color = Color.white;
        if (_poseText   != null) _poseText.color   = Color.white;

        if (_fkSolver != null && _fkSolver.tool0Transform != null && _fkSolver.tool0CameraTransform != null)
        {
            _tEEFCamera = _fkSolver.GetEEFCameraPose();
            Vector3 t   = _tEEFCamera.position;
            Vector3 eul = _tEEFCamera.rotation.eulerAngles;
            // Wrap to signed [-180, 180)
            eul.x = eul.x > 180f ? eul.x - 360f : eul.x;
            eul.y = eul.y > 180f ? eul.y - 360f : eul.y;
            eul.z = eul.z > 180f ? eul.z - 360f : eul.z;
            Debug.Log($"[CalibrationHUD] Simulated T_EEF_camera (hand-eye from URDF hierarchy):\n" +
                      $"  translation : ({t.x:+0.0000;-0.0000}, {t.y:+0.0000;-0.0000}, {t.z:+0.0000;-0.0000}) m\n" +
                      $"  rotation    : ({eul.x:+0.0;-0.0}\u00b0, {eul.y:+0.0;-0.0}\u00b0, {eul.z:+0.0;-0.0}\u00b0)  [X=pitch Y=yaw Z=roll]");
        }
        else
        {
            Debug.LogWarning("[CalibrationHUD] RobotFKSolver not found or missing tool0/camera transforms – T_EEF_camera unavailable.");
        }

        // Cache baseline layout so we can grow/shrink relative to the original authored sizes.
        if (_poseText != null)
            _basePoseTextHeight = _poseText.rectTransform.rect.height;
        if (_hudPanel != null)
            _baseHudPanelHeight = _hudPanel.rect.height;
        if (_imuText != null)
            _baseImuTextHeight = _imuText.rectTransform.rect.height;

    }

    void Update()
    {
        if (Keyboard.current != null && Keyboard.current.f9Key.wasPressedThisFrame)
            SetVisible(!_visible);

    }

    void LateUpdate()
    {
        if (!_visible) return;

        // Force styles every frame - TMP serialised vertex colour alpha can override
        // values set in Awake before the first frame is drawn.
        ForceTextStyles();

        UpdateFeed();
        string tagBlock = BuildTagPoseText();
        string imuBlock = BuildImuText();
        // _poseText and _imuText may be the same GameObject - write once
        if (_poseText != null)
            _poseText.text = tagBlock + (tagBlock.Length > 0 ? "\n" : "") + imuBlock;
        else if (_imuText != null)
            _imuText.text = tagBlock + (tagBlock.Length > 0 ? "\n" : "") + imuBlock;
        AutoResizeHud();
        if (_autoResizeHud) AutoResizeImuPanel();
    }

    // Enforces white colour, font size, overflow, and rich-text on every text element
    // each frame so Inspector/serialisation values can never clobber them at runtime.
    void ForceTextStyles()
    {
        ForceStyle(_statusText, 22);
        ForceStyle(_poseText,   24);
        ForceStyle(_imuText,    24);
    }

    static void ForceStyle(TMPro.TextMeshProUGUI tmp, int fontSize)
    {
        if (tmp == null) return;
        tmp.enableVertexGradient = false;
        tmp.colorGradient        = new TMPro.VertexGradient(Color.white);
        tmp.color                = Color.white;
        tmp.faceColor            = Color.white;
        tmp.fontSize             = fontSize;
        tmp.fontStyle            = TMPro.FontStyles.Normal;
        tmp.richText             = true;
        tmp.enableWordWrapping   = false;
        tmp.overflowMode         = TMPro.TextOverflowModes.Overflow;
        tmp.SetAllDirty();
    }

    // IMU Diagnostics

    string BuildImuText()
    {
        if (_imuSubscriber == null) return "";

        var   sb   = new System.Text.StringBuilder(256);
        bool  pose = _imuSubscriber.PoseValid;
        float hz   = _imuSubscriber.IMUReceiveHz;

        string hzCol  = hz  > 100f ? "#44FF88" : hz > 10f ? "#FFAA00" : "#FF5555";
        string fixCol = pose        ? "#44FF88" :                        "#FF5555";
        string mode   = _imuSubscriber.IsHandHeld ? "FREE-HAND" : "ARM-MOUNTED";

        sb.AppendLine($"<b>IMU</b>  {mode}  <color={hzCol}>{hz:F0} Hz</color>  <color={fixCol}>{(pose ? "POSE OK" : "NO FIX")}</color>");

        // Accelerometer - only gravity check (orientation health indicator)
        Vector3 acc  = _imuSubscriber.LastAccelUnity;
        float   gMag = acc.magnitude;
        bool    upright = Mathf.Abs(acc.y - 9.81f) < 1.5f && Mathf.Abs(acc.x) < 2f && Mathf.Abs(acc.z) < 2f;
        string  aCol  = upright ? "#44FF88" : "#FFAA00";
        string  aTick = upright ? "\u2713" : "\u26a0";
        sb.AppendLine($"  accel  <color={aCol}>{aTick}  X{acc.x:+0.00;-0.00}  Y{acc.y:+0.00;-0.00}  Z{acc.z:+0.00;-0.00}  m/s\u00b2  |g|={gMag:F2}</color>");

        // Orientation only - position from MEMS dead-reckoning is meaningless past ~1 s
        Vector3 eul = _imuSubscriber.FilterEulerAngles;
        sb.AppendLine($"  rot    P{eul.x:+0.0;-0.0}\u00b0  Y{eul.y:+0.0;-0.0}\u00b0  R{eul.z:+0.0;-0.0}\u00b0");

        // Anchor tag visibility
        bool tagVis = false;
        if (_imuSubscriber.detection?.DetectedTags != null)
            foreach (var tag in _imuSubscriber.detection.DetectedTags)
                if (tag.ID == _imuSubscriber.targetTagId) { tagVis = true; break; }
        string ancCol = tagVis ? "#44FF88" : "#FFAA00";
        sb.Append($"  anchor #{_imuSubscriber.targetTagId}  <color={ancCol}>{(tagVis ? "VISIBLE" : "LAST FIX")}</color>");
        return sb.ToString();
    }

    // Grows the IMU text rect and the HUD panel to fit the current content.
    void AutoResizeImuPanel()
    {
        if (_imuText == null) return;

        // Force TMP to compute layout so preferredHeight is up-to-date.
        _imuText.ForceMeshUpdate();

        float needed = _imuText.preferredHeight + 8f;   // 8 px bottom padding
        var   imuRt  = _imuText.rectTransform;

        // Resize the text element itself.
        imuRt.sizeDelta = new Vector2(imuRt.sizeDelta.x, needed);

        // Grow (never shrink below authored size) the HUD panel by however
        // much the IMU text overflows its originally authored height.
        if (_hudPanel != null)
        {
            float extra = Mathf.Max(0f, needed - _baseImuTextHeight);
            var   sd    = _hudPanel.sizeDelta;
            _hudPanel.sizeDelta = new Vector2(sd.x, _baseHudPanelHeight + extra);
        }
    }

    // Feed texture

    void UpdateFeed()
    {
        if (_feedImage == null || _colourSource == null) return;
        var tex = _colourSource.ColorTexture;
        if (tex == null) return;

        _feedImage.texture = tex;
        // SimpleImageSubscriber already flips vertically (flipVertically = true by default).
        // Use the default uvRect to avoid double-flipping.
        _feedImage.uvRect = new Rect(0f, 0f, 1f, 1f);
    }

    // Tag overlays: corner markers + pose text

    string BuildTagPoseText()
    {
        if (_detection == null) return "";

        var tags       = _detection.DetectedTags;
        var intrinsics = _detection.Intrinsics;

        var tagList = new List<AprilTag.TagPose>();
        if (tags != null)
            foreach (var t in tags) tagList.Add(t);

        int tagCount = tagList.Count;

        // Status strip
        if (_statusText != null)
        {
            _statusText.text = tagCount == 0 ? "No tags detected" :
                               tagCount == 1  ? "1 tag visible"   :
                               $"{tagCount} tags visible";
        }

        // Corner markers and axis lines
        EnsureMarkers(tagCount * 4);
        foreach (var m in _markerPool) m.gameObject.SetActive(false);
        EnsureAxisLines(tagCount * 3);
        foreach (var a in _axisLinePool) a.gameObject.SetActive(false);

        // Pose text
        var sb  = new System.Text.StringBuilder(512);
        bool canProject = intrinsics.IsValid && _feedImage != null;

        for (int i = 0; i < tagCount; i++)
        {
            var tag  = tagList[i];
            Vector3 cp  = tag.Position;                             // camera-frame translation
            float   dist = cp.magnitude;
            Vector3 ce  = SignedEuler(tag.Rotation.eulerAngles);

            sb.AppendLine($"<b>TAG {tag.ID}</b>");

            // Camera frame (camera is the origin)
            sb.AppendLine($"  CAM  t  {cp.x:+0.000;-0.000}  {cp.y:+0.000;-0.000}  {cp.z:+0.000;-0.000} m   d={dist:0.00} m");
            sb.AppendLine($"       r  P{ce.x:+0.0;-0.0}\u00b0  Y{ce.y:+0.0;-0.0}\u00b0  R{ce.z:+0.0;-0.0}\u00b0");

            // EEF-relative pose: T_EEF_tag = T_EEF_cam * T_cam_tag
            // Gives the tag position in the coordinate frame of the virtual end-effector.
            Pose? worldPose = _detection.GetDepthRefinedWorldPose(tag.ID);
            if (_fkSolver != null && _fkSolver.tool0CameraTransform != null && worldPose.HasValue)
            {
                Transform camTF = _detection.CameraTransform;
                if (camTF != null)
                {
                    Transform eefTF = _fkSolver.tool0Transform;
                    if (eefTF != null)
                    {
                        // Express the world-space tag pose in EEF local space
                        Vector3    ep  = eefTF.InverseTransformPoint(worldPose.Value.position);
                        Quaternion er  = Quaternion.Inverse(eefTF.rotation) * worldPose.Value.rotation;
                        Vector3    ee  = SignedEuler(er.eulerAngles);
                        sb.AppendLine($"  EEF  t  {ep.x:+0.000;-0.000}  {ep.y:+0.000;-0.000}  {ep.z:+0.000;-0.000} m");
                        sb.AppendLine($"       r  P{ee.x:+0.0;-0.0}\u00b0  Y{ee.y:+0.0;-0.0}\u00b0  R{ee.z:+0.0;-0.0}\u00b0");
                    }
                }
            }
            else if (_fkSolver == null)
            {
                sb.AppendLine("  EEF  assign _fkSolver in Inspector");
            }

            // Robot base frame
            if (_robotBase != null && worldPose.HasValue)
            {
                Pose    bp  = PoseEstimation.WorldToBase(worldPose.Value, _robotBase);
                Vector3 bpe = SignedEuler(bp.rotation.eulerAngles);
                sb.AppendLine($"  BASE t  {bp.position.x:+0.000;-0.000}  {bp.position.y:+0.000;-0.000}  {bp.position.z:+0.000;-0.000} m");
                sb.AppendLine($"       r  P{bpe.x:+0.0;-0.0}\u00b0  Y{bpe.y:+0.0;-0.0}\u00b0  R{bpe.z:+0.0;-0.0}\u00b0");
            }
            else if (_robotBase == null)
            {
                sb.AppendLine("  BASE  assign _robotBase in Inspector");
            }

            if (i < tagCount - 1) sb.AppendLine();

            // Corner markers
            if (!canProject) continue;
            float h = _detection.TagSize * 0.5f;
            var localCorners = new[]
            {
                new Vector3(-h,  h, 0f), new Vector3( h,  h, 0f),
                new Vector3( h, -h, 0f), new Vector3(-h, -h, 0f),
            };
            Rect feedRect = _feedImage.rectTransform.rect;
            for (int c = 0; c < 4; c++)
            {
                Vector3 camCorner = tag.Position + tag.Rotation * localCorners[c];
                Vector2 uv = PoseEstimation.ProjectToUV(camCorner, intrinsics);
                if (uv == Vector2.zero) continue;
                var marker = _markerPool[i * 4 + c];
                marker.gameObject.SetActive(true);
                marker.anchoredPosition = new Vector2((uv.x - 0.5f) * feedRect.width,
                                                      (0.5f - uv.y) * feedRect.height);
            }

            // Axis lines
            float axisLen = _detection.TagSize * _axisLengthFraction;
            Vector3 origin = tag.Position;
            Vector3[] axisTips =
            {
                origin + tag.Rotation * new Vector3(axisLen, 0f,       0f),
                origin + tag.Rotation * new Vector3(0f,      axisLen,  0f),
                origin + tag.Rotation * new Vector3(0f,      0f,      -axisLen),
            };
            Vector2 uvO = PoseEstimation.ProjectToUV(origin, intrinsics);
            if (uvO != Vector2.zero)
            {
                Vector2 p0 = new Vector2((uvO.x - 0.5f) * feedRect.width,
                                         (0.5f - uvO.y) * feedRect.height);
                for (int ax = 0; ax < 3; ax++)
                {
                    Vector2 uvT = PoseEstimation.ProjectToUV(axisTips[ax], intrinsics);
                    if (uvT == Vector2.zero) continue;
                    Vector2 p1 = new Vector2((uvT.x - 0.5f) * feedRect.width,
                                             (0.5f - uvT.y) * feedRect.height);
                    PlaceAxisLine(_axisLinePool[i * 3 + ax], p0, p1);
                }
            }
        }

        return tagCount > 0 ? sb.ToString() : "";
    }

    // Returns euler angles clamped to signed [-180, 180) range.
    static Vector3 SignedEuler(Vector3 e)
    {
        return new Vector3(
            e.x > 180f ? e.x - 360f : e.x,
            e.y > 180f ? e.y - 360f : e.y,
            e.z > 180f ? e.z - 360f : e.z);
    }

    void AutoResizeHud()
    {
        if (!_autoResizeHud) return;
        if (_poseText == null) return;

        // Ensure preferredHeight is up to date.
        Canvas.ForceUpdateCanvases();
        LayoutRebuilder.ForceRebuildLayoutImmediate(_poseText.rectTransform);

        float preferred = _poseText.preferredHeight;
        if (preferred <= 0f) return;

        // Resize pose text area
        _poseText.rectTransform.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical, preferred);

        // If the surrounding panel is a fixed size (no ContentSizeFitter), grow it too.
        if (_hudPanel != null && _baseHudPanelHeight > 0f && _basePoseTextHeight > 0f)
        {
            float delta = preferred - _basePoseTextHeight;
            float target = Mathf.Max(_baseHudPanelHeight + delta, _baseHudPanelHeight);
            _hudPanel.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical, target);
        }
    }

    // Marker pool management

    void EnsureMarkers(int required)
    {
        while (_markerPool.Count < required)
        {
            var go = new GameObject("CornerMarker", typeof(RectTransform), typeof(Image));
            var rt = go.GetComponent<RectTransform>();

            // Parent to the feed image so it moves with it
            rt.SetParent(_feedImage.rectTransform, false);
            rt.anchorMin = rt.anchorMax = new Vector2(0.5f, 0.5f); // centred anchor
            rt.pivot     = new Vector2(0.5f, 0.5f);
            rt.sizeDelta = new Vector2(_markerSize, _markerSize);

            go.GetComponent<Image>().color = _markerColour;
            go.SetActive(false);
            _markerPool.Add(rt);
        }
    }

    void EnsureAxisLines(int required)
    {
        while (_axisLinePool.Count < required)
        {
            int axisIndex = _axisLinePool.Count % 3;
            var go = new GameObject("AxisLine_" + "XYZ"[axisIndex], typeof(RectTransform), typeof(Image));
            var rt = go.GetComponent<RectTransform>();
            rt.SetParent(_feedImage.rectTransform, false);
            rt.anchorMin = rt.anchorMax = new Vector2(0.5f, 0.5f);
            rt.pivot     = new Vector2(0.5f, 0.5f);
            rt.sizeDelta = new Vector2(1f, _axisLineWidth); // width set per-frame by PlaceAxisLine
            go.GetComponent<Image>().color = AxisColors[axisIndex];
            go.SetActive(false);
            _axisLinePool.Add(rt);
        }
    }

    /// Positions and rotates a RectTransform to act as a line between two UI-space points.
    void PlaceAxisLine(RectTransform rt, Vector2 from, Vector2 to)
    {
        Vector2 dir    = to - from;
        float   length = dir.magnitude;
        if (length < 1f) { rt.gameObject.SetActive(false); return; }
        rt.gameObject.SetActive(true);
        rt.anchoredPosition = (from + to) * 0.5f;
        rt.sizeDelta        = new Vector2(length, _axisLineWidth);
        rt.localRotation    = Quaternion.Euler(0f, 0f, Mathf.Atan2(dir.y, dir.x) * Mathf.Rad2Deg);
    }

    public void SetVisible(bool show)
    {
        _visible = show;
        if (_hudPanel != null)
            _hudPanel.gameObject.SetActive(show);
    }
}
