// Author: Jackson Russell

using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

/// Applies engine-wide performance settings at startup.
/// Execution order -1000 ensures this runs before all other scripts.
[DefaultExecutionOrder(-1000)]
public class AppInit : MonoBehaviour
{
    [Header("Frame Rate")]
    [Tooltip("Target frame rate in Hz. Set to match your display / XR headset refresh rate.")]
    public int targetFrameRate = 90;

    // Auto-create + early frame-rate lock 
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    static void Bootstrap()
    {
        // Apply frame pacing immediately before any frame renders.
#if UNITY_EDITOR
        QualitySettings.vSyncCount  = 1;
        Application.targetFrameRate = -1;
#else
        QualitySettings.vSyncCount  = 0;
        Application.targetFrameRate = 90; // default; Awake will override from inspector
#endif

        if (FindAnyObjectByType<AppInit>() != null) return; // already in scene
        var go = new GameObject("[AppInit]");
        go.AddComponent<AppInit>();
        DontDestroyOnLoad(go);
    }

    [Header("Testing")]
    [Tooltip("Disable all kinematic solver components at startup so you can benchmark the "
           + "camera/rendering path in isolation. Affects JacobianIKSolver, EEFTargetController, "
           + "and RobotFKSolver.")]
    public bool disableKinematics = true; // set false to re-enable solvers

    [Header("Shadows")]
    [Tooltip("Shadow draw distance in metres. Default Unity value is 40m which forces cascade "
           + "recalculation every step you take (crossing the ~13m cascade boundary). "
           + "For a robot workspace scene 6-8m is sufficient and eliminates walk-lag.")]
    public float shadowDistance = 6f;

    void Awake()
    {
        // Frame rate
#if UNITY_EDITOR
        // In the editor a soft targetFrameRate cap produces frame-time variance
        // (each frame takes a different number of ms), which shows up as jitter.
        // VSync locks delivery to a fixed hardware interval - zero variance.
        // The editor Game view "VSync" checkbox does the same thing but only for
        // the Game window; setting it here applies globally and survives domain reloads.
        QualitySettings.vSyncCount  = 1;   // lock to monitor refresh (144 Hz on your display)
        Application.targetFrameRate = -1;  // ignored when vSyncCount > 0
#else
        // On the XR device the headset runtime owns frame pacing (ATW/ASW).
        // Disable Unity VSync so the runtime can drive at its own rate (90 Hz).
        QualitySettings.vSyncCount  = 0;
        Application.targetFrameRate = targetFrameRate;
#endif

        // Align the physics (FixedUpdate) tick to the render rate.
        // Default fixedDeltaTime = 0.02 (50 Hz). At 90fps this means FixedUpdate fires
        // at uneven intervals relative to rendered frames (1-2 ticks per Update, alternating).
        // JacobianIKSolver runs in FixedUpdate - the uneven firing creates irregular arm
        // updates which are visually obvious as jitter during camera movement.
        Time.fixedDeltaTime    = 1f / targetFrameRate;

        // Cap the maximum catch-up if a frame takes longer than expected.
        // Without this, a single long frame causes FixedUpdate to fire many times
        // in the next Update to "catch up", spiking CPU for an entire frame.
        Time.maximumDeltaTime  = 2f / targetFrameRate; // allow 1 missed tick max

        // Shadows
        // QualitySettings.shadowDistance is IGNORED by URP - the pipeline reads its
        // own asset value. Must set it via the URP asset directly.
        QualitySettings.shadowDistance = shadowDistance; // fallback for non-URP
        var urpAsset = GraphicsSettings.currentRenderPipeline as UniversalRenderPipelineAsset;
        if (urpAsset != null)
            urpAsset.shadowDistance = shadowDistance;

        Debug.Log($"[AppInit] targetFrameRate={targetFrameRate}  "
                + $"fixedDeltaTime={Time.fixedDeltaTime:F4}  "
                + $"shadowDistance={shadowDistance}m");

        // Kinematics bypass
        if (disableKinematics)
        {
            int count = 0;
            count += DisableAll<JacobianIKSolver>();
            count += DisableAll<EEFTargetController>();
            count += DisableAll<RobotFKSolver>();
            Debug.Log($"[AppInit] disableKinematics=true — disabled {count} kinematic component(s).");
        }
    }

    static int DisableAll<T>() where T : MonoBehaviour
    {
        T[] found = FindObjectsByType<T>(FindObjectsSortMode.None);
        foreach (T c in found) c.enabled = false;
        return found.Length;
    }
}
