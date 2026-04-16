// Author: Jackson Russell

using System.Collections;
using Unity.XR.CoreUtils;
using UnityEngine;

// Bridges the XRI Starter Assets XR Origin rig with the rest of the project.
// Resolves Left/Right Controller Transforms on Start. IsXRActive is true whenever
// the rig is present and controllers are found - covers both a real headset and
// the XR Interaction Simulator in the Editor.
//
// Setup: assign the XR Origin (XR Rig) GameObject to xrOrigin in the Inspector.
public class XRRigSetup : MonoBehaviour
{
    [Header("XRI Starter Assets Rig")]
    [Tooltip("Assign the 'XR Origin (XR Rig)' GameObject from the scene.")]
    public XROrigin xrOrigin;

    public static XRRigSetup Instance      { get; private set; }
    public bool              IsXRActive    { get; private set; }
    public Transform         LeftController  { get; private set; }
    public Transform         RightController { get; private set; }

    void Awake()
    {
        Instance = this;
    }

    IEnumerator Start()
    {
        // Wait one frame so the XRI rig fully initialises.
        yield return null;

        if (xrOrigin == null)
            xrOrigin = Object.FindFirstObjectByType<XROrigin>();

        if (xrOrigin == null)
        {
            Debug.LogWarning("[XRRigSetup] XROrigin not found - XR input disabled.");
            yield break;
        }

        LeftController  = FindChildByName(xrOrigin.transform, "Left Controller");
        RightController = FindChildByName(xrOrigin.transform, "Right Controller");

        if (LeftController  == null) Debug.LogWarning("[XRRigSetup] 'Left Controller' not found in XR Origin hierarchy.");
        if (RightController == null) Debug.LogWarning("[XRRigSetup] 'Right Controller' not found in XR Origin hierarchy.");

        // Active whenever the right controller is found - covers real headset and simulator.
        IsXRActive = RightController != null;

        Debug.Log($"[XRRigSetup] IsXRActive={IsXRActive}  Left={LeftController?.name}  Right={RightController?.name}");
    }

    // Depth-first search for a child with the given name.
    static Transform FindChildByName(Transform root, string name)
    {
        foreach (Transform child in root)
        {
            if (child.name == name) return child;
            var found = FindChildByName(child, name);
            if (found != null) return found;
        }
        return null;
    }
}

