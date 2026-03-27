using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.InputSystem;
using TMPro;

/// Displays the D455 colour feed in a corner overlay, draws 2D tag corner markers
/// projected onto the feed image, and shows per-tag pose estimation results.
///
/// Wiring (Inspector):
///   _colourSource  -> ImageSubscriber (SimpleImageSubscriber)
///   _detection     -> CalibrationManager (Detection)
///   _feedImage     -> CameraFeed (RawImage)
///   _statusText    -> TagInfoText (TMP)
///   _poseText      -> PoseText (TMP)
///   _hudPanel      -> CameraHUD (RectTransform)
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

    [Header("Panel Visibility")]
    [SerializeField] bool _visible = true;
    // Toggle the HUD panel with F9 (hardcoded to avoid Key enum serialisation mismatch)
    [SerializeField] RectTransform _hudPanel;

    // Pooled corner marker RectTransforms; expanded on demand.
    // Each detected tag uses 4 consecutive entries.
    readonly List<RectTransform> _markerPool = new List<RectTransform>();

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

        if (_feedImage  == null) Debug.LogWarning("[CalibrationHUD] CameraFeed RawImage not found.");
        if (_poseText   == null) Debug.LogWarning("[CalibrationHUD] PoseText TMP not found.");
        if (_statusText == null) Debug.LogWarning("[CalibrationHUD] TagInfoText TMP not found.");
    }

    void Update()
    {
        if (Keyboard.current != null && Keyboard.current.f9Key.wasPressedThisFrame)
            SetVisible(!_visible);
    }

    void LateUpdate()
    {
        if (!_visible) return;

        UpdateFeed();
        UpdateTagOverlays();
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

    void UpdateTagOverlays()
    {
        if (_detection == null) return;

        var tags    = _detection.DetectedTags;
        var intrinsics = _detection.Intrinsics;
        var camTF   = _detection.CameraTransform;

        // Build per-tag data in one pass
        var tagList  = new List<AprilTag.TagPose>();
        if (tags != null)
            foreach (var t in tags) tagList.Add(t);

        int tagCount = tagList.Count;

        // status text
        if (_statusText != null)
            _statusText.text = tagCount == 0 ? "No tags detected"
                             : tagCount == 1 ? "1 tag detected"
                             : $"{tagCount} tags detected";

        // corner markers
        EnsureMarkers(tagCount * 4);
        // Hide all first, re-enable only the active ones
        foreach (var m in _markerPool) m.gameObject.SetActive(false);

        // pose text lines
        var poseLines = new System.Text.StringBuilder();

        bool canProject = intrinsics.IsValid && _feedImage != null;

        for (int i = 0; i < tagCount; i++)
        {
            var tag = tagList[i];

            // pose readou
            // tag.Position / tag.Rotation are camera-space (metres, Unity coords: Z forward)
            Vector3 pos  = tag.Position;
            float   dist = pos.magnitude;

            // Fetch depth-refined world pose if depth is available
            Pose? worldPose = _detection.GetDepthRefinedWorldPose(tag.ID);
            string depthNote = worldPose.HasValue ? "" : " (mono)";

            // Euler angles (degrees) from the camera-space rotation returned by the detector  
            // Unity eulerAngles are in [0, 360), convert to signed [-180, 180)
            // so the display reads naturally (e.g. -5deg rather than 355deg).
            Vector3 eul = PoseEstimation.PoseToEulerDeg(new Pose(pos, tag.Rotation));
            eul.x = eul.x > 180f ? eul.x - 360f : eul.x;
            eul.y = eul.y > 180f ? eul.y - 360f : eul.y;
            eul.z = eul.z > 180f ? eul.z - 360f : eul.z;

            poseLines.AppendLine(
                $"ID {tag.ID}  pos ({pos.x:+0.000;-0.000}, {pos.y:+0.000;-0.000}, {pos.z:0.000})m  d={dist:0.00}m{depthNote}");
            poseLines.AppendLine(
                $"       rot ({eul.x:+0.0;-0.0}\u00b0, {eul.y:+0.0;-0.0}\u00b0, {eul.z:+0.0;-0.0}\u00b0)  [X=pitch Y=yaw Z=roll]");

            // corner markers
            if (!canProject) continue;

            // 4 corners of the tag in tag-local space (flat, z=0)
            float h = _detection.TagSize * 0.5f;
            var localCorners = new[]
            {
                new Vector3(-h,  h, 0f),
                new Vector3( h,  h, 0f),
                new Vector3( h, -h, 0f),
                new Vector3(-h, -h, 0f),
            };

            Rect feedRect = _feedImage.rectTransform.rect;

            for (int c = 0; c < 4; c++)
            {
                // Corner in camera space
                Vector3 camCorner = tag.Position + tag.Rotation * localCorners[c];

                // Project to normalised image UV [0,1]
                Vector2 uv = PoseEstimation.ProjectToUV(camCorner, intrinsics);
                if (uv == Vector2.zero) continue;

                // Map UV to local position inside the RawImage RectTransform.
                // The feed is displayed with uvRect (0,1,1,-1): texture-top -> display-top.
                // UV (0,0) = image-top = display-top (y = +feedRect.height/2).
                float lx = (uv.x - 0.5f) * feedRect.width;
                float ly = (0.5f - uv.y) * feedRect.height;   // invert Y for Unity rect

                var marker = _markerPool[i * 4 + c];
                marker.gameObject.SetActive(true);
                marker.anchoredPosition = new Vector2(lx, ly);
            }
        }

        if (_poseText != null)
            _poseText.text = tagCount > 0 ? poseLines.ToString() : "";
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

    public void SetVisible(bool show)
    {
        _visible = show;
        if (_hudPanel != null)
            _hudPanel.gameObject.SetActive(show);
    }
}
