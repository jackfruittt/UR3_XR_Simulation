using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using TMPro;

public class SimpleJointController : MonoBehaviour
{
    [Header("UI References")]
    public Slider[] jointSliders = new Slider[6];
    public TMP_InputField[] jointInputFields = new TMP_InputField[6];
    public Button publishButton;
    public Button homeButton;  // Optional: button to return to home position
    
    [Header("Robot Reference")]
    public UR3SourceDestinationPublisher robotPublisher;
    
    private float[] currentJointAngles = new float[6];
    private Vector2[] jointLimits = new Vector2[6]; // Read from sliders
    private bool initialized = false;
    
    void Start()
    {
        if (initialized) return; // Prevent double initialization
        
        Debug.Log("SimpleJointController Start() called");
        
        // Initialize sliders - use configured min/max from Inspector
        for (int i = 0; i < 6; i++)
        {
            if (jointSliders[i] != null)
            {
                // Read limits from slider (set in Inspector)
                jointLimits[i] = new Vector2(jointSliders[i].minValue, jointSliders[i].maxValue);
                Debug.Log($"Slider {i}: min={jointSliders[i].minValue}, max={jointSliders[i].maxValue}, interactable={jointSliders[i].interactable}");
                
                // Don't set to 0 - let UpdateFromRobot() sync to actual robot position
                
                // Remove any existing listeners first
                jointSliders[i].onValueChanged.RemoveAllListeners();
                
                int index = i; // Capture for closure
                jointSliders[i].onValueChanged.AddListener((value) => OnSliderChanged(index, value));
                Debug.Log($"Added listener to slider {i}");
            }
            else
            {
                Debug.LogWarning($"Slider {i} is NULL!");
            }
            
            if (jointInputFields[i] != null)
            {
                jointInputFields[i].text = "0";
                int index = i; // Capture for closure
                jointInputFields[i].onEndEdit.AddListener((value) => OnInputFieldChanged(index, value));
            }
        }
        
        // Setup publish button
        if (publishButton != null)
        {
            publishButton.onClick.AddListener(PublishJointPositions);
        }
        
        // Setup home button
        if (homeButton != null)
        {
            homeButton.onClick.AddListener(MoveToHome);
        }
        
        Debug.Log($"SimpleJointController initialized. robotPublisher: {(robotPublisher != null ? "Found" : "NULL")}");
        Debug.Log("=== ALL SLIDERS INITIALIZED - TRY DRAGGING ONE NOW ===");
        
        // Sync sliders to robot's home position after a short delay
        if (robotPublisher != null)
        {
            StartCoroutine(SyncSlidersAfterInit());
        }
        
        initialized = true;
    }
    
    System.Collections.IEnumerator SyncSlidersAfterInit()
    {
        // Wait for robot to reach home position
        yield return new WaitForSeconds(0.2f);
        UpdateFromRobot();
        Debug.Log("Sliders synced to robot home position");
    }
    
    void OnSliderChanged(int jointIndex, float value)
    {
        Debug.Log($"Slider {jointIndex} changed to: {value}");
        currentJointAngles[jointIndex] = value;
        
        // Update corresponding input field
        if (jointInputFields[jointIndex] != null)
        {
            jointInputFields[jointIndex].text = value.ToString("F2");
        }
        
        // Update robot joint directly (local control, independent of ROS)
        if (robotPublisher != null)
        {
            robotPublisher.SetJointAngleLocally(jointIndex, value);
        }
    }
    
    void OnInputFieldChanged(int jointIndex, string text)
    {
        if (float.TryParse(text, out float value))
        {
            // Clamp to limits
            value = Mathf.Clamp(value, jointLimits[jointIndex].x, jointLimits[jointIndex].y);
            currentJointAngles[jointIndex] = value;
            
            // Update corresponding slider
            if (jointSliders[jointIndex] != null)
            {
                jointSliders[jointIndex].SetValueWithoutNotify(value);
            }
            
            // Update input field with clamped value
            jointInputFields[jointIndex].text = value.ToString("F2");
            
            // Update robot joint directly (local control, independent of ROS)
            if (robotPublisher != null)
            {
                robotPublisher.SetJointAngleLocally(jointIndex, value);
            }
        }
        else
        {
            // Reset to current value if invalid
            jointInputFields[jointIndex].text = currentJointAngles[jointIndex].ToString("F2");
        }
    }
    
    public void PublishJointPositions()
    {
        if (robotPublisher != null)
        {
            robotPublisher.PublishJointCommand(currentJointAngles);
            Debug.Log($"Published joint positions: [{string.Join(", ", currentJointAngles)}]");
        }
        else
        {
            Debug.LogError("Robot publisher reference is not set!");
        }
    }
    
    public void MoveToHome()
    {
        if (robotPublisher != null)
        {
            robotPublisher.MoveToHomePosition();
            // Update sliders to reflect home position
            StartCoroutine(SyncSlidersAfterInit());
            Debug.Log("Moved to home position");
        }
        else
        {
            Debug.LogError("Robot publisher reference is not set!");
        }
    }
    
    public void UpdateFromRobot()
    {
        Debug.Log("UpdateFromRobot() called");
        if (robotPublisher != null)
        {
            float[] robotAngles = robotPublisher.GetCurrentJointAngles();
            if (robotAngles != null && robotAngles.Length == 6)
            {
                Debug.Log($"Got robot angles: [{string.Join(", ", robotAngles)}]");
                for (int i = 0; i < 6; i++)
                {
                    currentJointAngles[i] = robotAngles[i];
                    if (jointSliders[i] != null)
                    {
                        jointSliders[i].SetValueWithoutNotify(robotAngles[i]);
                    }
                    if (jointInputFields[i] != null)
                    {
                        jointInputFields[i].text = robotAngles[i].ToString("F2");
                    }
                }
            }
            else
            {
                Debug.LogWarning("robotAngles is null or wrong length");
            }
        }
    }
    
    // Individual joint control methods for button-based control
    public void SetJoint(int jointIndex, float angle)
    {
        if (jointIndex >= 0 && jointIndex < 6)
        {
            currentJointAngles[jointIndex] = Mathf.Clamp(angle, jointLimits[jointIndex].x, jointLimits[jointIndex].y);
            if (jointSliders[jointIndex] != null)
            {
                jointSliders[jointIndex].SetValueWithoutNotify(currentJointAngles[jointIndex]);
            }
        }
    }
}
