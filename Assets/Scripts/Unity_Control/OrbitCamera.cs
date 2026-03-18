using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Orbit camera controller for navigating around your scene
/// - Left Mouse Drag: Rotate around target
/// - Right Mouse Drag: Pan camera
/// - Scroll Wheel: Zoom in/out
/// - WASD: Move target point
/// - Q/E: Move up/down
/// - F: Focus on origin
/// 
/// Supports both Old Input Manager and New Input System
/// </summary>
public class OrbitCamera : MonoBehaviour
{
    [Header("Target Settings")]
    [Tooltip("Point the camera orbits around")]
    public Vector3 targetPosition = Vector3.zero;
    
    [Tooltip("Automatically find and focus on objects in scene")]
    public bool autoFocusOnStart = false;
    
    [Header("Orbit Controls")]
    [Tooltip("Mouse sensitivity for rotation")]
    [Range(0.1f, 5f)]
    public float orbitSpeed = 1.0f;
    
    [Tooltip("Smooth rotation damping")]
    [Range(0f, 0.99f)]
    public float rotationDamping = 0.9f;
    
    [Header("Zoom Settings")]
    [Tooltip("Current distance from target")]
    public float distance = 5f;
    
    [Tooltip("Zoom speed")]
    [Range(0.1f, 5f)]
    public float zoomSpeed = 1f;
    
    [Tooltip("Minimum zoom distance")]
    public float minDistance = 0.5f;
    
    [Tooltip("Maximum zoom distance")]
    public float maxDistance = 50f;
    
    [Tooltip("Smooth zoom damping")]
    [Range(0f, 0.99f)]
    public float zoomDamping = 0.9f;
    
    [Header("Pan Settings")]
    [Tooltip("Mouse pan sensitivity")]
    [Range(0.1f, 5f)]
    public float panSpeed = 0.5f;
    
    [Header("Keyboard Controls")]
    [Tooltip("WASD movement speed")]
    [Range(0.1f, 10f)]
    public float keyboardMoveSpeed = 2f;
    
    [Tooltip("Q/E vertical movement speed")]
    [Range(0.1f, 10f)]
    public float verticalMoveSpeed = 2f;
    
    [Header("Input Settings")]
    [Tooltip("Invert vertical rotation")]
    public bool invertY = false;
    
    [Tooltip("Mouse button for orbit (0=Left, 1=Right, 2=Middle)")]
    public int orbitMouseButton = 0;
    
    [Tooltip("Mouse button for pan (0=Left, 1=Right, 2=Middle)")]
    public int panMouseButton = 1;
    
    // Private variables
    private float currentRotationX = 0f;
    private float currentRotationY = 0f;
    private float targetRotationX = 0f;
    private float targetRotationY = 0f;
    private float currentDistance;
    private float targetDistance;
    private Vector3 velocity = Vector3.zero;
    
    void Start()
    {
        // Calculate initial orbit parameters from current camera transform
        Vector3 directionToCamera = transform.position - targetPosition;
        float actualDistance = directionToCamera.magnitude;
        
        // If distance is set to a reasonable value in inspector, use it; otherwise use actual distance
        if (distance <= 0 || Mathf.Abs(distance - 5f) < 0.01f) // 5f is default value
        {
            distance = actualDistance;
        }
        
        currentDistance = actualDistance;
        targetDistance = actualDistance;
        
        // Calculate rotation angles from the direction vector
        if (actualDistance > 0.001f)
        {
            Vector3 normalizedDir = directionToCamera.normalized;
            
            // Calculate horizontal angle (Y rotation)
            currentRotationX = Mathf.Atan2(normalizedDir.x, normalizedDir.z) * Mathf.Rad2Deg;
            
            // Calculate vertical angle (X rotation)
            currentRotationY = -Mathf.Asin(normalizedDir.y) * Mathf.Rad2Deg;
        }
        else
        {
            // Fallback to transform angles if camera is too close to target
            Vector3 angles = transform.eulerAngles;
            currentRotationX = angles.y;
            currentRotationY = angles.x;
        }
        
        targetRotationX = currentRotationX;
        targetRotationY = currentRotationY;
        
        // Auto-focus on scene objects if requested
        if (autoFocusOnStart)
        {
            FocusOnSceneCenter();
        }
        
        Debug.Log($"[OrbitCamera] Initialized at position {transform.position}, distance: {currentDistance:F2}, rotation: ({currentRotationX:F1}, {currentRotationY:F1})");
    }
    
    void LateUpdate()
    {
        HandleMouseInput();
        HandleKeyboardInput();
        HandleZoomInput();
        
        // Smooth damping for rotation
        currentRotationX = Mathf.Lerp(currentRotationX, targetRotationX, 1f - rotationDamping);
        currentRotationY = Mathf.Lerp(currentRotationY, targetRotationY, 1f - rotationDamping);
        
        // Smooth damping for zoom
        currentDistance = Mathf.Lerp(currentDistance, targetDistance, 1f - zoomDamping);
        
        // Calculate camera position
        Quaternion rotation = Quaternion.Euler(currentRotationY, currentRotationX, 0);
        Vector3 position = rotation * new Vector3(0, 0, -currentDistance) + targetPosition;
        
        // Apply to camera
        transform.position = position;
        transform.LookAt(targetPosition);
    }
    
    void HandleMouseInput()
    {
#if ENABLE_INPUT_SYSTEM
        // New Input System
        var mouse = Mouse.current;
        if (mouse == null) return;
        
        bool leftMousePressed = mouse.leftButton.isPressed;
        bool rightMousePressed = mouse.rightButton.isPressed;
        Vector2 mouseDelta = mouse.delta.ReadValue();
        
        // Orbit rotation
        if ((orbitMouseButton == 0 && leftMousePressed) || 
            (orbitMouseButton == 1 && rightMousePressed) ||
            (orbitMouseButton == 2 && mouse.middleButton.isPressed))
        {
            float sensitivity = 0.1f; // Adjust for new input system
            float mouseX = mouseDelta.x * sensitivity;
            float mouseY = mouseDelta.y * sensitivity;
            
            targetRotationX += mouseX * orbitSpeed;
            targetRotationY += mouseY * orbitSpeed * (invertY ? 1 : -1);
            
            // Clamp vertical rotation to avoid flipping
            targetRotationY = Mathf.Clamp(targetRotationY, -89f, 89f);
        }
        
        // Pan movement
        if ((panMouseButton == 0 && leftMousePressed) || 
            (panMouseButton == 1 && rightMousePressed) ||
            (panMouseButton == 2 && mouse.middleButton.isPressed))
        {
            float sensitivity = 0.005f; // Adjust for new input system
            float mouseX = mouseDelta.x * sensitivity;
            float mouseY = mouseDelta.y * sensitivity;
            
            // Calculate pan direction based on camera orientation
            Vector3 right = transform.right;
            Vector3 up = transform.up;
            
            // Pan relative to current view
            Vector3 panMovement = (-right * mouseX + -up * mouseY) * panSpeed * currentDistance;
            targetPosition += panMovement;
        }
#else
        // Old Input Manager
        // Orbit rotation
        if (Input.GetMouseButton(orbitMouseButton))
        {
            float mouseX = Input.GetAxis("Mouse X");
            float mouseY = Input.GetAxis("Mouse Y");
            
            targetRotationX += mouseX * orbitSpeed * 3f;
            targetRotationY += mouseY * orbitSpeed * 3f * (invertY ? 1 : -1);
            
            // Clamp vertical rotation to avoid flipping
            targetRotationY = Mathf.Clamp(targetRotationY, -89f, 89f);
        }
        
        // Pan movement
        if (Input.GetMouseButton(panMouseButton))
        {
            float mouseX = Input.GetAxis("Mouse X");
            float mouseY = Input.GetAxis("Mouse Y");
            
            // Calculate pan direction based on camera orientation
            Vector3 right = transform.right;
            Vector3 up = transform.up;
            
            // Pan relative to current view
            Vector3 panMovement = (-right * mouseX + -up * mouseY) * panSpeed * currentDistance * 0.1f;
            targetPosition += panMovement;
        }
#endif
    }
    
    void HandleKeyboardInput()
    {
#if ENABLE_INPUT_SYSTEM
        var keyboard = Keyboard.current;
        if (keyboard == null) return;
        
        Vector3 moveDirection = Vector3.zero;
        
        // WASD movement (relative to camera orientation, on ground plane)
        if (keyboard.wKey.isPressed)
        {
            Vector3 forward = transform.forward;
            forward.y = 0;
            forward.Normalize();
            moveDirection += forward;
        }
        if (keyboard.sKey.isPressed)
        {
            Vector3 forward = transform.forward;
            forward.y = 0;
            forward.Normalize();
            moveDirection -= forward;
        }
        if (keyboard.aKey.isPressed)
        {
            moveDirection -= transform.right;
        }
        if (keyboard.dKey.isPressed)
        {
            moveDirection += transform.right;
        }
        
        // Q/E for vertical movement
        if (keyboard.qKey.isPressed)
        {
            moveDirection += Vector3.down * verticalMoveSpeed;
        }
        if (keyboard.eKey.isPressed)
        {
            moveDirection += Vector3.up * verticalMoveSpeed;
        }
        
        // Apply movement
        if (moveDirection.magnitude > 0)
        {
            targetPosition += moveDirection.normalized * keyboardMoveSpeed * Time.deltaTime;
        }
        
        // Focus on origin with F key
        if (keyboard.fKey.wasPressedThisFrame)
        {
            FocusOnOrigin();
        }
        
        // Reset to default view with R key
        if (keyboard.rKey.wasPressedThisFrame)
        {
            ResetView();
        }
#else
        Vector3 moveDirection = Vector3.zero;
        
        // WASD movement (relative to camera orientation, on ground plane)
        if (Input.GetKey(KeyCode.W))
        {
            Vector3 forward = transform.forward;
            forward.y = 0;
            forward.Normalize();
            moveDirection += forward;
        }
        if (Input.GetKey(KeyCode.S))
        {
            Vector3 forward = transform.forward;
            forward.y = 0;
            forward.Normalize();
            moveDirection -= forward;
        }
        if (Input.GetKey(KeyCode.A))
        {
            moveDirection -= transform.right;
        }
        if (Input.GetKey(KeyCode.D))
        {
            moveDirection += transform.right;
        }
        
        // Q/E for vertical movement
        if (Input.GetKey(KeyCode.Q))
        {
            moveDirection += Vector3.down * verticalMoveSpeed;
        }
        if (Input.GetKey(KeyCode.E))
        {
            moveDirection += Vector3.up * verticalMoveSpeed;
        }
        
        // Apply movement
        if (moveDirection.magnitude > 0)
        {
            targetPosition += moveDirection.normalized * keyboardMoveSpeed * Time.deltaTime;
        }
        
        // Focus on origin with F key
        if (Input.GetKeyDown(KeyCode.F))
        {
            FocusOnOrigin();
        }
        
        // Reset to default view with R key
        if (Input.GetKeyDown(KeyCode.R))
        {
            ResetView();
        }
#endif
    }
    
    void HandleZoomInput()
    {
#if ENABLE_INPUT_SYSTEM
        var mouse = Mouse.current;
        if (mouse == null) return;
        
        float scroll = mouse.scroll.ReadValue().y;
        // Normalize scroll value (new input system gives larger values)
        scroll *= 0.001f;
#else
        float scroll = Input.GetAxis("Mouse ScrollWheel");
#endif
        
        if (Mathf.Abs(scroll) > 0.01f)
        {
            targetDistance -= scroll * zoomSpeed * currentDistance * 0.5f;
            targetDistance = Mathf.Clamp(targetDistance, minDistance, maxDistance);
        }
    }
    
    /// <summary>
    /// Focus camera on the origin (0,0,0)
    /// </summary>
    public void FocusOnOrigin()
    {
        targetPosition = Vector3.zero;
        Debug.Log("[OrbitCamera] Focused on origin");
    }
    
    /// <summary>
    /// Reset camera to default view
    /// </summary>
    public void ResetView()
    {
        targetRotationX = 0f;
        targetRotationY = 30f;
        targetDistance = 5f;
        targetPosition = Vector3.zero;
        Debug.Log("[OrbitCamera] View reset");
    }
    
    /// <summary>
    /// Focus on a specific point
    /// </summary>
    public void FocusOnPoint(Vector3 point)
    {
        targetPosition = point;
    }
    
    /// <summary>
    /// Focus on a specific GameObject
    /// </summary>
    public void FocusOnObject(GameObject obj)
    {
        if (obj != null)
        {
            Bounds bounds = CalculateBounds(obj);
            targetPosition = bounds.center;
            
            // Auto-adjust distance based on object size
            float objectSize = bounds.size.magnitude;
            targetDistance = Mathf.Clamp(objectSize * 2f, minDistance, maxDistance);
            
            Debug.Log($"[OrbitCamera] Focused on {obj.name}");
        }
    }
    
    /// <summary>
    /// Auto-focus on the center of all renderers in the scene
    /// </summary>
    void FocusOnSceneCenter()
    {
        Renderer[] renderers = FindObjectsOfType<Renderer>();
        
        if (renderers.Length == 0)
        {
            Debug.Log("[OrbitCamera] No renderers found, using origin");
            return;
        }
        
        // Calculate bounding box of all renderers
        Bounds combinedBounds = renderers[0].bounds;
        foreach (Renderer renderer in renderers)
        {
            combinedBounds.Encapsulate(renderer.bounds);
        }
        
        targetPosition = combinedBounds.center;
        
        // Auto-adjust distance based on scene size
        float sceneSize = combinedBounds.size.magnitude;
        targetDistance = Mathf.Clamp(sceneSize * 1.5f, minDistance, maxDistance);
        
        Debug.Log($"[OrbitCamera] Auto-focused on scene center at {targetPosition}");
    }
    
    /// <summary>
    /// Calculate bounds of a GameObject and all its children
    /// </summary>
    Bounds CalculateBounds(GameObject obj)
    {
        Renderer[] renderers = obj.GetComponentsInChildren<Renderer>();
        
        if (renderers.Length == 0)
        {
            return new Bounds(obj.transform.position, Vector3.one);
        }
        
        Bounds bounds = renderers[0].bounds;
        foreach (Renderer renderer in renderers)
        {
            bounds.Encapsulate(renderer.bounds);
        }
        
        return bounds;
    }
    
    void OnDrawGizmosSelected()
    {
        // Draw target position
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(targetPosition, 0.1f);
        
        // Draw orbit path
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(targetPosition, currentDistance);
    }
}
