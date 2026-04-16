// Author: Jackson Russell

using UnityEngine;

/// Draws an FPS-style crosshair at screen centre plus a singularity flash banner.
/// Telemetry display is handled by HUDController (UI Toolkit).
public class CrosshairHUD : MonoBehaviour
{
    [Header("Crosshair Colours")]
    public Color colorIdle        = new Color(1f, 1f, 1f, 0.85f);
    public Color colorAimed       = new Color(1f, 1f, 0f, 1f);
    public Color colorControlling = new Color(1f, 0.4f, 0f, 1f);

    [Header("Crosshair Geometry")]
    [Tooltip("Half-length of each arm of the cross (pixels)")]
    public float crossArmLength = 14f;
    [Tooltip("Thickness of each arm (pixels)")]
    public float crossThickness = 2f;
    [Tooltip("Pixel gap at the centre to leave empty")]
    public float gapSize = 4f;
    [Tooltip("Radius of the ring shown when aimed at target")]
    public float aimRingRadius = 18f;

    [Header("Singularity Banner")]
    public Color singularityColor = new Color(1f, 0.15f, 0.15f, 0.9f);
    [Tooltip("Flash frequency when a singularity is active (Hz)")]
    public float flashFrequency = 4f;

    private EEFTargetController _target;
    private JacobianIKSolver    _ikSolver;
    private GUIStyle            _labelStyle;
    private GUIStyle            _warnStyle;
    private bool                _stylesInitialised;

    private string                             _singLabel   = "";
    private SingularityChecker.SingularityType _lastSingType =
        SingularityChecker.SingularityType.None;

    void Start()
    {
        _target   = EEFTargetController.ActiveInstance
                 ?? FindObjectOfType<EEFTargetController>();
        _ikSolver = FindObjectOfType<JacobianIKSolver>();
    }

    void Update()
    {
        var stype = _ikSolver != null
                  ? _ikSolver.LastSingularityType
                  : SingularityChecker.SingularityType.None;
        if (stype != _lastSingType)
        {
            _lastSingType = stype;
            _singLabel    = stype != SingularityChecker.SingularityType.None
                          ? $"SINGULARITY -- {stype.ToString().ToUpper()}"
                          : "";
        }
    }

    void EnsureStyles()
    {
        if (_stylesInitialised) return;
        _stylesInitialised = true;

        _labelStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize  = 30,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleCenter,
        };
        _labelStyle.normal.textColor = Color.white;

        _warnStyle = new GUIStyle(GUI.skin.box)
        {
            fontSize  = 30,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleCenter,
        };
        _warnStyle.normal.textColor = Color.white;
    }

    void OnGUI()
    {
        EnsureStyles();

        // Crosshair, aim ring, and singularity banner only draw in FPS cursor-locked mode.
        if (Cursor.lockState != CursorLockMode.Locked) return;

        float cx = Screen.width  * 0.5f;
        float cy = Screen.height * 0.5f;

        bool controlling = _target != null && _target.IsControllingTarget;
        bool aimed       = _target != null && _target.IsInCrosshairRange;

        Color color = controlling ? colorControlling
                    : aimed       ? colorAimed
                    :               colorIdle;

        GUI.color = color;

        float arm   = crossArmLength;
        float thick = crossThickness;
        float gap   = gapSize;

        // Horizontal arms
        GUI.DrawTexture(new Rect(cx - arm - gap * 0.5f, cy - thick * 0.5f,
                                 arm, thick), Texture2D.whiteTexture);
        GUI.DrawTexture(new Rect(cx + gap * 0.5f, cy - thick * 0.5f,
                                 arm, thick), Texture2D.whiteTexture);

        // Vertical arms
        GUI.DrawTexture(new Rect(cx - thick * 0.5f, cy - arm - gap * 0.5f,
                                 thick, arm), Texture2D.whiteTexture);
        GUI.DrawTexture(new Rect(cx - thick * 0.5f, cy + gap * 0.5f,
                                 thick, arm), Texture2D.whiteTexture);

        // Centre dot while controlling
        if (controlling)
        {
            float dotR = 3f;
            GUI.DrawTexture(new Rect(cx - dotR, cy - dotR, dotR * 2f, dotR * 2f),
                            Texture2D.whiteTexture);
        }

        // Aim ring
        if (aimed || controlling)
            DrawWireCircle(cx, cy, aimRingRadius, color, 24);

        // Context hint
        string hint = controlling ? "LMB  Release  |  Mouse=drag  Scroll=depth"
                    : aimed       ? "LMB  Grab target"
                    :               null;

        if (hint != null)
        {
            GUI.color = Color.white;
            _labelStyle.normal.textColor = color;
            GUI.Label(new Rect(cx - 120f, cy + aimRingRadius + 8f, 240f, 22f),
                      hint, _labelStyle);
        }

        GUI.color = Color.white;

        // Singularity flash banner
        if (!string.IsNullOrEmpty(_singLabel))
        {
            float t      = Mathf.PingPong(Time.unscaledTime * flashFrequency, 1f);
            Color banner = singularityColor;
            banner.a    *= Mathf.Lerp(0.5f, 1f, t);

            float bw = 360f, bh = 34f;
            float bx = cx - bw * 0.5f;
            float by = cy - Screen.height * 0.3f;

            GUI.color = banner;
            GUI.Box(new Rect(bx, by, bw, bh), GUIContent.none);

            GUI.color = Color.white;
            _warnStyle.normal.textColor = Color.white;
            GUI.Label(new Rect(bx, by, bw, bh), _singLabel, _warnStyle);
        }
    }

    /// Simple wire circle drawn with a series of rectangles at each sample point.
    void DrawWireCircle(float cx, float cy, float r, Color color, int segments)
    {
        float prev_x = cx + r, prev_y = cy;
        for (int i = 1; i <= segments; i++)
        {
            float angle = i * Mathf.PI * 2f / segments;
            float nx = cx + Mathf.Cos(angle) * r;
            float ny = cy + Mathf.Sin(angle) * r;

            DrawLine(prev_x, prev_y, nx, ny, 1.5f, color);
            prev_x = nx;
            prev_y = ny;
        }
    }

    void DrawLine(float x1, float y1, float x2, float y2, float width, Color col)
    {
        float dx = x2 - x1, dy = y2 - y1;
        float len = Mathf.Sqrt(dx * dx + dy * dy);
        if (len < 0.5f) return;

        GUI.color = col;
        GUIUtility.RotateAroundPivot(Mathf.Atan2(dy, dx) * Mathf.Rad2Deg,
                                     new Vector2(x1, y1));
        GUI.DrawTexture(new Rect(x1, y1 - width * 0.5f, len, width),
                        Texture2D.whiteTexture);
        GUI.matrix = Matrix4x4.identity;
    }
}
