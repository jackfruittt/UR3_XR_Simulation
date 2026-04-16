# UR3 XR Simulation

Unity 6 teleoperation and hand-eye calibration system for a UR3e robot arm with ROS2 integration, Intel RealSense D455 streaming, and an XR-ready interface.

## Project Goal

Teleoperate a physical UR3e through XR. The operator places a target puck in the robot's task space and the IK solver drives the robot to it. A hand-eye calibration pipeline aligns the RealSense camera frame to the robot base frame, enabling AR overlays of robot state onto the live camera feed.

## Roadmap

**Phase 1 -- Validate calibration pipeline** (current)  
Verify the full capture -> solve -> save loop works on real hardware with the physical robot and D455.

**Phase 2 -- Verify calibration result**  
Quantify reprojection error, visualise the calibrated transform in-scene, confirm AR overlay alignment.

**Phase 3 -- XR migration**  
Move from desktop mouse input to Meta Quest controller ray-casting for puck placement and HUD interaction.

## Requirements

| Dependency | Version |
|---|---|
| Unity | 6000.3.9f1 |
| ROS2 | Humble |
| Camera | Intel RealSense D455 |
| Robot | UR3e |
| XR (Phase 3) | Meta Quest via Meta OpenXR |

See [SETUP.md](SETUP.md) for full dependency installation instructions.

## Quick Start

### 1. ROS2 side
```bash
# ROS-TCP endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# RealSense streaming
python3 d455_unity_streaming.launch.py
```

### 2. Unity side
1. Open `Assets/Scenes/GetStarted_Scene`
2. Set ROS IP in `Robotics -> ROS Settings`
3. Hit Play

## Controls

| Input | Action |
|---|---|
| Left-click in task space | Move robot target puck |
| F1 | HUD -- Robot tab |
| F2 | HUD -- Calibration tab |
| F9 | Toggle HUD |
| Left drag | Orbit camera |
| Right drag | Pan camera |
| Scroll | Zoom |

## Hand-Eye Calibration

Open the **CALIBRATION** tab in the HUD.

1. **Auto Collect** -- arm moves through pre-defined poses and captures pairs automatically
2. **Manual Collect** -- jog arm to a pose, click **Confirm Pose** to capture
3. **Finish + Solve** -- runs Tsai-Lenz AX=XB solver, saves `hand_eye_calibration.json` to `Application.persistentDataPath`
4. **Load JSON** -- reloads a previously saved calibration

Minimum 3 pairs required; 5+ recommended. Residual error (degrees) shown in HUD after solve.

To test the pipeline without a physical camera, enable **Pseudo Detection** on the `HandEyeCalibrationCollector` component. The collector synthesises tag poses from FK so the full capture -> solver -> save chain can be verified using only MoveIt joint states.

## Project Structure

```
Assets/
  Scenes/
    GetStarted_Scene          Main scene
  Scripts/
    App/
      AppInit.cs              Engine-wide startup settings
    Calibration/
      Detection.cs            AprilTag detection (ROS colour feed)
      HandEyeCalibrationCollector.cs
      HandEyeSolver.cs        Tsai-Lenz AX=XB solver
      CalibrationResult.cs    JSON serialisation
      PoseEstimation.cs       Camera-space pose utilities
      TagDrawer.cs            3D tag overlays
      Util.cs                 ROS/Unity frame conversion
    Camera/
      FirstPersonCamera.cs
    Kinematics/
      JacobianIKSolver.cs     RMRC + gradient descent IK
      FKSolver.cs             Transform-hierarchy FK
      EEFTargetController.cs  Puck placement + arc trajectory
      SelfCollisionAvoider.cs Potential field avoidance
      SingularityChecker.cs
    ROS/
      RealSense/
        ROSPointCloudRenderer.cs  GPU point cloud
        SimpleImageSubscriber.cs
        IMUSubscriber.cs          Madgwick filter (6-DOF)
        D455Anchor.cs
      ur3e/
        UR3SourceDestinationPublisher.cs
        UR3ROSHandler.cs          Centralised ROS interface
    UI/
      HUDController.cs
      CalibrationHUD.cs
      CrosshairHUD.cs
    XR/
      XRRigSetup.cs
      WristHUDController.cs     World-space wrist panel
  UI/
    HUD.uxml / HUD.uss
    WristHUD.uxml / WristHUD.uss
  ur_e_description/             Robot URDF
```

## RealSense Diagnostics

```bash
# Confirm USB 3 connection (required for depth + colour at full rate)
./check_realsense_usb.sh
```

## Author

Jackson Russell
