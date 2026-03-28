using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// First-person camera controller simulating a 1.78m tall person standing near the UR3e.
///
/// Controls:
///   Left-click (or auto on Start) - locks cursor for mouse look
///   Escape                        - unlocks cursor (e.g. to interact with UI sliders)
///   Mouse move                    - look around (yaw on rig, pitch on camera head)
///   WASD                          - walk
///   Left Shift + WASD             - run.
public class FirstPersonCamera : MonoBehaviour
{
    [Header("Look Settings")]
    [Tooltip("Mouse sensitivity for looking")]
    [Range(0.1f, 10f)]
    public float mouseSensitivity = 2f;

    [Tooltip("Maximum vertical look angle in degrees")]
    [Range(30f, 90f)]
    public float verticalClamp = 80f;

    [Header("Movement Settings")]
    [Tooltip("Walking speed (m/s)")]
    [Range(0.5f, 10f)]
    public float walkSpeed = 2f;

    [Tooltip("Running speed (m/s), hold Left Shift to sprint")]
    [Range(1f, 20f)]
    public float runSpeed = 4f;

    [Header("References")]
    [Tooltip("The Camera child Transform that pitches up/down independently of the body")]
    public Transform cameraHead;

    // private state
    private float pitchAngle = 0f;
    private bool cursorLocked = false;

    void Start()
    {
        // Auto-find camera head if not assigned
        if (cameraHead == null)
        {
            Camera cam = GetComponentInChildren<Camera>();
            if (cam != null)
                cameraHead = cam.transform;
            else
                Debug.LogWarning("[FirstPersonCamera] No Camera child found; assign cameraHead in Inspector.");
        }

        LockCursor();
    }

    void Update()
    {
        HandleCursorLock();

        if (cursorLocked)
        {
            HandleLook();
            HandleMovement();
        }
    }

    // Cursor management

    void LockCursor()
    {
        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;
        cursorLocked = true;
    }

    void UnlockCursor()
    {
        Cursor.lockState = CursorLockMode.None;
        Cursor.visible = true;
        cursorLocked = false;
    }

    void HandleCursorLock()
    {
#if ENABLE_INPUT_SYSTEM
        var keyboard = Keyboard.current;
        var mouse    = Mouse.current;
        if (keyboard == null || mouse == null) return;

        if (keyboard.escapeKey.wasPressedThisFrame)
            UnlockCursor();
        else if (!cursorLocked && mouse.leftButton.wasPressedThisFrame)
            LockCursor();
#else
        if (Input.GetKeyDown(KeyCode.Escape))
            UnlockCursor();
        else if (!cursorLocked && Input.GetMouseButtonDown(0))
            LockCursor();
#endif
    }

    // Look (mouse)

    void HandleLook()
    {
        float mouseX, mouseY;

#if ENABLE_INPUT_SYSTEM
        var mouse = Mouse.current;
        if (mouse == null) return;

        // Raw delta is in pixels; scale down to match the Old Input axis range
        Vector2 delta = mouse.delta.ReadValue() * 0.05f;
        mouseX = delta.x * mouseSensitivity;
        mouseY = delta.y * mouseSensitivity;
#else
        mouseX = Input.GetAxis("Mouse X") * mouseSensitivity;
        mouseY = Input.GetAxis("Mouse Y") * mouseSensitivity;
#endif

        // Yaw - rotate the whole rig around world up
        transform.Rotate(Vector3.up, mouseX, Space.World);

        // Pitch - tilt only the camera head
        if (cameraHead != null)
        {
            pitchAngle -= mouseY;
            pitchAngle = Mathf.Clamp(pitchAngle, -verticalClamp, verticalClamp);
            cameraHead.localRotation = Quaternion.Euler(pitchAngle, 0f, 0f);
        }
    }

    // Movement (keyboard)

    void HandleMovement()
    {
        Vector3 move  = Vector3.zero;
        float   speed = walkSpeed;

#if ENABLE_INPUT_SYSTEM
        var keyboard = Keyboard.current;
        if (keyboard == null) return;

        if (keyboard.wKey.isPressed) move += transform.forward;
        if (keyboard.sKey.isPressed) move -= transform.forward;
        if (keyboard.aKey.isPressed) move -= transform.right;
        if (keyboard.dKey.isPressed) move += transform.right;
        if (keyboard.leftShiftKey.isPressed) speed = runSpeed;
#else
        if (Input.GetKey(KeyCode.W)) move += transform.forward;
        if (Input.GetKey(KeyCode.S)) move -= transform.forward;
        if (Input.GetKey(KeyCode.A)) move -= transform.right;
        if (Input.GetKey(KeyCode.D)) move += transform.right;
        if (Input.GetKey(KeyCode.LeftShift)) speed = runSpeed;
#endif

        // Keep movement on the horizontal plane regardless of look direction
        move.y = 0f;

        if (move.sqrMagnitude > 0f)
            transform.position += move.normalized * speed * Time.deltaTime;
    }
}
