using UnityEngine;
using UnityEngine.UI;

public class SimpleJointControllerTest : MonoBehaviour
{
    public Slider[] jointSliders = new Slider[6];
    
    void Start()
    {
        Debug.Log("=== SimpleJointControllerTest Start ===");
        
        for (int i = 0; i < 6; i++)
        {
            if (jointSliders[i] != null)
            {
                Debug.Log($"Setting up slider {i}: interactable={jointSliders[i].interactable}, gameObject.active={jointSliders[i].gameObject.activeInHierarchy}");
                
                int index = i; // Capture for closure
                jointSliders[i].onValueChanged.AddListener((value) => {
                    Debug.Log($"*** SLIDER {index} CHANGED TO: {value} ***");
                });
            }
            else
            {
                Debug.LogWarning($"Slider {i} is NULL!");
            }
        }
        
        Debug.Log("=== ALL SLIDERS READY - TRY DRAGGING NOW ===");
    }
}
