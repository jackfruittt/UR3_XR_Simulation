using UnityEngine;

/// Interactive EEF target controller.
/// Click on the target marker to grab it, then drag with the mouse to move
/// it within the robot's hemispherical workspace. Scroll wheel adjusts height.
///
/// This positions a target only - IK is required to drive the joints to it.
/// Attach to any scene GameObject and wire up references in the Inspector.
public class EEFTargetController : MonoBehaviour
{
    [Header("References")]
    public RobotFKSolver fkSolver;
    public Transform robotBase;

    [Header("Workspace")]
    [Tooltip("UR3e max reach in metres (default: 0.50)")]
    public float maxReach = 0.50f;

    [Header("Interaction")]
    [Tooltip("World-space radius around target marker that counts as a click hit")]
    public float grabRadius = 0.06f;
    [Tooltip("Scroll wheel vertical sensitivity")]
    public float verticalSensitivity = 0.3f;

    [Header("Visuals")]
    [Tooltip("Colour of the target marker sphere")]
    public Color markerColor = new Color(0f, 1f, 0.5f, 0.8f);
    [Tooltip("Diameter of the target marker sphere")]
    public float markerSize = 0.04f;

    // The current target position in world space. Read by the IK solver.
    public Vector3 TargetPosition { get; private set; }

    // The current target orientation. Seeded from EEF orientation on each grab.
    public Quaternion TargetRotation { get; private set; }

    private GameObject _marker;
    private bool _isDragging;
    private Plane _dragPlane;
    private Camera _cam;

    void Start()
    {
        _cam = Camera.main;

        // Seed target at current EEF position and orientation
        TargetPosition = fkSolver != null ? fkSolver.GetEEFPosition() : robotBase.position + Vector3.up * 0.3f;
        TargetRotation = fkSolver != null ? fkSolver.GetEEFRotation() : Quaternion.identity;

        _marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        _marker.name = "EEF_TargetMarker";
        _marker.transform.localScale = Vector3.one * markerSize;
        Destroy(_marker.GetComponent<Collider>());

        var rend = _marker.GetComponent<Renderer>();
        var mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
        mat.color = markerColor;
        rend.material = mat;

        _marker.transform.position = TargetPosition;
    }

    void Update()
    {
        if (_cam == null) return;

        if (Input.GetMouseButtonDown(0))
            TryGrab();

        if (Input.GetMouseButtonUp(0))
            _isDragging = false;

        if (_isDragging)
            DragTarget();
    }

    void TryGrab()
    {
        Ray ray = _cam.ScreenPointToRay(Input.mousePosition);

        // Closest distance from ray to target marker centre
        float dist = Vector3.Cross(ray.direction, TargetPosition - ray.origin).magnitude;
        if (dist > grabRadius) return;

        _isDragging = true;
        // Snap orientation target to current EEF orientation at the moment of grab
        if (fkSolver != null) TargetRotation = fkSolver.GetEEFRotation();
        // Drag on a camera-facing plane at the target's current depth
        _dragPlane = new Plane(-_cam.transform.forward, TargetPosition);
    }

    void DragTarget()
    {
        Ray ray = _cam.ScreenPointToRay(Input.mousePosition);
        if (!_dragPlane.Raycast(ray, out float enter)) return;

        Vector3 worldPoint = ray.GetPoint(enter);

        // Scroll wheel shifts vertically
        worldPoint += Vector3.up * (Input.mouseScrollDelta.y * verticalSensitivity * Time.deltaTime * 60f);

        // --- Hemisphere constraint ---
        Vector3 localPos = worldPoint - robotBase.position;

        // Enforce above-base floor (y >= 0)
        localPos.y = Mathf.Max(0f, localPos.y);

        // Clamp to max reach sphere
        if (localPos.magnitude > maxReach)
            localPos = localPos.normalized * maxReach;

        TargetPosition = robotBase.position + localPos;
        _marker.transform.position = TargetPosition;
    }

    // Draw workspace hemisphere in the Scene view for reference
    void OnDrawGizmosSelected()
    {
        if (robotBase == null) return;

        Gizmos.color = new Color(0f, 0.8f, 1f, 0.15f);
        Gizmos.DrawSphere(robotBase.position, maxReach);

        Gizmos.color = new Color(0f, 0.8f, 1f, 0.4f);
        Gizmos.DrawWireSphere(robotBase.position, maxReach);
    }
}
