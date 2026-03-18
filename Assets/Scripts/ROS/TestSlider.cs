using UnityEngine;
using UnityEngine.UI;

public class TestSlider : MonoBehaviour
{
    void Start()
    {
        Slider slider = GetComponent<Slider>();
        if (slider != null)
        {
            Debug.Log($"TestSlider: Found slider, interactable={slider.interactable}, min={slider.minValue}, max={slider.maxValue}");
            slider.onValueChanged.AddListener(OnValueChanged);
        }
        else
        {
            Debug.LogError("TestSlider: No Slider component found!");
        }
    }
    
    void OnValueChanged(float value)
    {
        Debug.Log($"TestSlider changed to: {value}");
    }
}
