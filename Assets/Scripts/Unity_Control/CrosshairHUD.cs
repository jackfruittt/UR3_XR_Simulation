using UnityEngine;

/// Draws an FPS-style crosshair at screen centre plus context-sensitive IK feedback.
///
/// Appearance:
///   - Crosshair is only visible while the cursor is locked (FirstPersonCamera mode).
///   - Idle: thin white cross.
///   - Aimed (IsInCrosshairRange): dot + ring, turns yellow,  "RMB / E  Grab target" hint.
///   - Controlling (IsControllingTarget): filled dot, turns orange, "RMB / E  Release" hint.
///   - Singularity warning: red flashing banner when the IK solver detects a singularity.
///
/// Auto-finds EEFTargetController and JacobianIKSolver at Start.
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

    [Header("Status Panel")]
    [Tooltip("Show the robot telemetry panel in the top-left corner.")]
    public bool showStatusPanel = true;

    // Private
    private EEFTargetController  _target;
    private JacobianIKSolver     _ikSolver;
    private GUIStyle             _labelStyle;
    private GUIStyle             _warnStyle;
    private GUIStyle             _panelHeaderStyle;
    private GUIStyle             _panelLabelStyle;
    private GUIStyle             _panelValueStyle;
    private bool                 _stylesInitialised;

    void Start()
    {
        _target   = EEFTargetController.ActiveInstance
                 ?? FindObjectOfType<EEFTargetController>();
        _ikSolver = FindObjectOfType<JacobianIKSolver>();
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

        _panelHeaderStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize  = 30,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleLeft,
        };
        _panelHeaderStyle.normal.textColor = new Color(1f, 1f, 1f, 0.40f);

        _panelLabelStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize  = 30,
            fontStyle = FontStyle.Normal,
            alignment = TextAnchor.MiddleLeft,
        };
        _panelLabelStyle.normal.textColor = new Color(0.58f, 0.58f, 0.58f, 1f);

        _panelValueStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize  = 30,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleLeft,
        };
        _panelValueStyle.normal.textColor = Color.white;
    }

    void OnGUI()
    {
        EnsureStyles();

        // Status panel is always visible during Play, regardless of cursor mode.
        if (showStatusPanel)
            DrawStatusPanel();

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
        DrawSingularityBanner(cx, cy);
    }

    // Flash a red banner at the top of the screen when a singularity is detected.
    // Manipulability and IK blend info are shown in the status panel instead.
    void DrawSingularityBanner(float cx, float cy)
    {
        if (_ikSolver == null) return;

        var type = _ikSolver.LastSingularityType;
        if (type == SingularityChecker.SingularityType.None) return;

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
        GUI.Label(new Rect(bx, by, bw, bh),
                  $"SINGULARITY -- {type.ToString().ToUpper()}",
                  _warnStyle);
    }

    // Status panel - top-left corner, always visible during Play.
    // Show EEF and target positions relative to the robot base, position error,
    // reach percentage, IK solver blend state, and current control state.
    void DrawStatusPanel()
    {
        if (_target == null) return;

        const float panelX  = 16f;
        const float panelY  = 16f;
        const float panelW  = 1020f;
        const float lineH   = 44f;
        const float pad     = 20f;
        const float divGap  = 12f;
        const float labelW  = 120f;
        float       labelX  = panelX + pad;
        float       valueX  = labelX + labelW;
        // Value column extends to the panel right edge minus one pad so text never clips.
        float       valueW  = panelX + panelW - pad - valueX;

        // 7 data rows + header + 3 dividers
        float panelH = 8f * lineH + 3f * divGap + pad * 2f;

        GUI.color = new Color(0f, 0f, 0f, 0.60f);
        GUI.DrawTexture(new Rect(panelX, panelY, panelW, panelH), Texture2D.whiteTexture);
        GUI.color = Color.white;

        float y = panelY + pad;

        // Header
        GUI.Label(new Rect(labelX, y, panelW - pad * 2f, lineH), "ROBOT STATUS", _panelHeaderStyle);
        y += lineH;
        DrawHRule(labelX, y - 1f, panelW - pad * 2f);
        y += divGap;

        // EEF position (base frame)
        Vector3 eefWorld = (_target.fkSolver != null) ? _target.fkSolver.GetEEFPosition() : Vector3.zero;
        Vector3 eefLocal = (_target.robotBase != null) ? eefWorld - _target.robotBase.position : eefWorld;
        DrawRow(labelX, valueX, valueW, y, lineH, "EEF",
                $"X {eefLocal.x:+0.000;-0.000}   Y {eefLocal.y:+0.000;-0.000}   Z {eefLocal.z:+0.000;-0.000} m",
                new Color(0.45f, 0.90f, 1.00f));
        y += lineH;

        // Target position (base frame)
        Vector3 tgtLocal = (_target.robotBase != null)
                         ? _target.TargetPosition - _target.robotBase.position
                         : _target.TargetPosition;
        Color tgtColor   = _target.IsControllingTarget ? colorControlling
                         : _target.IsInCrosshairRange  ? colorAimed
                         :                               Color.white;
        DrawRow(labelX, valueX, valueW, y, lineH, "TARGET",
                $"X {tgtLocal.x:+0.000;-0.000}   Y {tgtLocal.y:+0.000;-0.000}   Z {tgtLocal.z:+0.000;-0.000} m",
                tgtColor);
        y += lineH;
        DrawHRule(labelX, y, panelW - pad * 2f);
        y += divGap;

        // Position error and radial position within the work ring
        float errM      = (_target.TargetPosition - eefWorld).magnitude;
        float ringNorm  = _target.TableNormalisedRadius();
        Color errColor  = errM < 0.005f ? new Color(0.30f, 1.00f, 0.42f)
                        : errM < 0.025f ? new Color(1.00f, 0.85f, 0.20f)
                        :                 new Color(1.00f, 0.40f, 0.30f);
        DrawRow(labelX, valueX, valueW, y, lineH, "ERROR",
                $"{errM * 1000f:F1} mm     RING  {ringNorm * 100f:F0}%",
                errColor);
        y += lineH;
        DrawHRule(labelX, y, panelW - pad * 2f);
        y += divGap;

        // IK solver blend state
        if (_ikSolver != null)
        {
            float  beta    = _ikSolver.LastBlendFactor;
            float  manip   = _ikSolver.LastManipulability;
            string modeStr = beta > 0.5f ? "DLS" : "GD ";
            Color  ikColor = Color.Lerp(new Color(1.00f, 0.38f, 0.26f),
                                        new Color(0.30f, 0.92f, 0.42f), beta);
            DrawRow(labelX, valueX, valueW, y, lineH, "IK",
                    $"{modeStr}   w = {manip:F4}   beta = {beta:F2}",
                    ikColor);
            y += lineH;

            // Joint velocity row: peak % of pi + per-joint magnitudes.
            float[] vels    = _ikSolver.LastJointVelocitiesRad;
            float   peak    = 0f;
            if (vels != null)
                for (int j = 0; j < vels.Length; j++)
                    if (Mathf.Abs(vels[j]) > peak) peak = Mathf.Abs(vels[j]);
            float  peakPct  = peak / Mathf.PI * 100f;
            Color  velColor = Color.Lerp(new Color(0.30f, 0.92f, 0.42f),
                                         new Color(1.00f, 0.38f, 0.26f),
                                         Mathf.Clamp01(peakPct / 100f));
            string velStr;
            if (vels != null && vels.Length == 6)
                velStr = $"peak {peakPct:F0}%   "
                       + $"{Mathf.Abs(vels[0]):F2} "
                       + $"{Mathf.Abs(vels[1]):F2} "
                       + $"{Mathf.Abs(vels[2]):F2} "
                       + $"{Mathf.Abs(vels[3]):F2} "
                       + $"{Mathf.Abs(vels[4]):F2} "
                       + $"{Mathf.Abs(vels[5]):F2} rad/s";
            else
                velStr = "--";
            DrawRow(labelX, valueX, valueW, y, lineH, "VEL", velStr, velColor);
            y += lineH;
        }

        // Control state, also reflects singularity escape phase
        string stateStr;
        Color  stateColor;
        if (_ikSolver != null && _ikSolver.CurrentSolverState == JacobianIKSolver.SolverState.EscapingToHome)
        {
            stateStr  = "HOMING";
            stateColor = new Color(1.00f, 0.60f, 0.10f);   // amber
        }
        else if (_ikSolver != null && _ikSolver.CurrentSolverState == JacobianIKSolver.SolverState.ResumeDelay)
        {
            stateStr  = "RESUMING";
            stateColor = new Color(0.90f, 0.90f, 0.20f);   // yellow
        }
        else
        {
            stateStr  = _target.IsControllingTarget ? "CONTROLLING"
                      : _target.IsInCrosshairRange  ? "AIMED"
                      :                               "IDLE";
            stateColor = _target.IsControllingTarget ? colorControlling
                       : _target.IsInCrosshairRange  ? colorAimed
                       :                               new Color(0.50f, 0.50f, 0.50f);
        }
        DrawRow(labelX, valueX, valueW, y, lineH, "STATE", stateStr, stateColor);
    }

    void DrawRow(float lx, float vx, float vw, float y, float h, string label, string value, Color valueColor)
    {
        GUI.color = Color.white;
        _panelLabelStyle.normal.textColor = new Color(0.58f, 0.58f, 0.58f, 1f);
        GUI.Label(new Rect(lx, y, vx - lx - 4f, h), label, _panelLabelStyle);
        _panelValueStyle.normal.textColor = valueColor;
        GUI.Label(new Rect(vx, y, vw, h), value, _panelValueStyle);
    }

    void DrawHRule(float x, float y, float w)
    {
        GUI.color = new Color(1f, 1f, 1f, 0.10f);
        GUI.DrawTexture(new Rect(x, y, w, 1f), Texture2D.whiteTexture);
        GUI.color = Color.white;
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
