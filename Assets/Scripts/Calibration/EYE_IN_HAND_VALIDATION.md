# Eye-in-Hand Calibration Validation (Simulation-Only)

## Background

Eye-in-hand calibration solves for **X**, the fixed transform from the robot's
end-effector (tool0) to the camera mounted on it:

$$T_{tool0 \rightarrow camera}$$

The classic form is the **AX = XB** problem:

| Symbol | Meaning |
|--------|---------|
| **A** | Relative end-effector motion between two robot poses: $T_{EEF_i}^{-1} \cdot T_{EEF_j}$ |
| **B** | The corresponding relative target motion seen by the camera: $T_{cam_i \rightarrow tag}^{-1} \cdot T_{cam_j \rightarrow tag}$ |
| **X** | The unknown camera-to-tool0 transform (what we are solving for) |

With N robot poses you get N−1 independent equation pairs. Standard solvers
(Tsai–Lenz, Park–Martin, Horaud–Dornaika, etc.) find X in a least-squares sense.

---

## Why Validate in Simulation?

The real robot is unavailable, but a simulation can act as a perfect oracle:

- Unity knows the **exact** world-space pose of `tool0` and the camera at every
  frame — no encoder noise, no DH-parameter error.
- The camera-to-tool0 transform **X** is a known, fixed parent-child relationship
  in the scene hierarchy. You can read it directly.
- You can generate as many robot poses as you like and construct synthetic (A, B)
  pairs analytically, bypassing the need for physical motion or a real tag.

The validation is therefore: **give your solver synthetic data derived from a
known X, then check that the solver recovers that same X.**

---

## The Validation Workflow

### Step 1 — Identify the ground-truth X in the Unity scene

In the scene hierarchy, `X = T_{tool0 → camera}`:

```
UR3 robot
  └── tool0                 ← end-effector link
        └── D455_Camera     ← camera mount
```

`X` is simply the local Transform of `D455_Camera` relative to `tool0`.
In Unity:

```csharp
Matrix4x4 X_gt = tool0.worldToLocalMatrix * camera.localToWorldMatrix;
```

This gives you the 4×4 homogeneous matrix you will compare the solver's answer
against.

---

### Step 2 — Sample N robot poses

Move the simulated robot arm to N distinct configurations (≥ 5, ideally 10–15).
At each pose i, record:

- **$T_{EEF_i}$** — the world-space pose of `tool0`
  (`Util.GetEEFMatrix(tool0)`)
- **$T_{cam_i \rightarrow tag}$** — the pose of the calibration tag *in camera
  space*

For the simulation case you do not need the real camera stream.  
You already know the world-space pose of both the camera and the tag
`T_{world \rightarrow tag}`, so the camera-space tag pose is:

$$T_{cam_i \rightarrow tag} = T_{world \rightarrow camera_i}^{-1} \cdot T_{world \rightarrow tag}$$

In Unity:

```csharp
Matrix4x4 T_cam_to_tag = camera.worldToLocalMatrix * tagTransform.localToWorldMatrix;
```

---

### Step 3 — Build the (A, B) pairs

For each consecutive pair of poses (i, j):

$$A_{ij} = T_{EEF_i}^{-1} \cdot T_{EEF_j}$$

$$B_{ij} = T_{cam_i \rightarrow tag}^{-1} \cdot T_{cam_j \rightarrow tag}$$

These are the inputs to your AX = XB solver.  With clean simulation data,
every pair is consistent, so even 5 pairs should yield a near-perfect result.

---

### Step 4 — Run your solver

Feed the list of (A, B) pairs to your calibration code exactly as you would
with real data — the interface is identical. Your solver returns **X_solved**.

---

### Step 5 — Quantify the error

Compare **X_solved** to **X_gt** using two metrics:

**Rotation error** (angle between the two rotation matrices):

$$\theta_{err} = \arccos\!\left(\frac{\text{trace}(R_{gt}^T R_{solved}) - 1}{2}\right)$$

A value below ~0.1° is excellent for noise-free data.

**Translation error** (Euclidean distance):

$$d_{err} = \| t_{gt} - t_{solved} \|_2$$

Below ~0.1 mm is excellent for noise-free data.

---

## Checking Your Real AprilTag Detections (No Real Robot Needed)

The live RealSense camera can still contribute to validation even without the
robot moving:

1. **Hold the camera by hand** and move it to several positions while the
   calibration tag sits on a stable surface.
2. `poseEstimation.cs` / `detection.cs` give you live $T_{cam \rightarrow tag}$
   estimates from the D455 stream.
3. Compare consecutive detections: if the tag is stationary, the world-space tag
   position inferred via any assumed X should remain constant. Drift = error
   in X or in the detection pipeline.

This tests the detection half of the pipeline independently.

---

## Adding Noise to Stress-Test the Solver

Once the noise-free case passes, synthetic Gaussian noise can reveal robustness:

- **Rotation noise**: perturb each recorded EEF quaternion by a small random
  angle (e.g. σ = 0.5°).
- **Translation noise**: add zero-mean Gaussian noise to each EEF position
  (e.g. σ = 0.5 mm).
- **Detection noise**: add noise to $T_{cam \rightarrow tag}$ to simulate tag
  detection jitter.

Re-run the solver at increasing σ values and plot error vs noise level.
This tells you the minimum number of poses and the detection accuracy your
pipeline needs to stay within a target calibration tolerance.

---

## Summary Checklist

- [ ] Read ground-truth X from the scene hierarchy (`tool0 → D455_Camera`)
- [ ] Collect ≥ 5 robot poses in the simulation (spread across the workspace)
- [ ] Compute synthetic $(A_i, B_i)$ pairs using known world transforms
- [ ] Run your AX = XB solver on those pairs
- [ ] Assert rotation error < 0.1° and translation error < 0.1 mm (noise-free)
- [ ] Re-test with injected noise to characterise solver sensitivity
- [ ] (Optional) Verify live D455 detections are self-consistent with a
      stationary target
