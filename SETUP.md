# Setup Guide

Complete dependency installation for a fresh clone of this project.

---

## 1. System Requirements

- Ubuntu 22.04 (tested)
- Unity 6000.3.9f1
- ROS2 Humble
- Python 3.10+
- Git LFS (for large Unity assets)

---

## 2. ROS2 and Robot Dependencies

### ROS2 Humble

Follow the official installation guide:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

### ROS-TCP Endpoint

The Unity-ROS bridge. Clone the forked version and build in your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/jackfruittt/ROS-TCP-Endpoint.git
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash
```

### UR Robot Driver (real hardware only)

```bash
sudo apt install ros-humble-ur
```

### MoveIt 2 (simulation)

```bash
sudo apt install ros-humble-moveit
```

### RealSense ROS2 Driver

```bash
sudo apt install ros-humble-librealsense2* ros-humble-realsense2-*
```

Verify the camera connects at USB 3 speed before use:
```bash
./check_realsense_usb.sh
```

---

## 3. Unity Package Dependencies

Three packages are installed as **local file references** and must be cloned alongside the project before opening Unity. If these folders are missing, Unity will fail to compile.

### ROS-TCP Connector

```bash
cd ~/Software
git clone https://github.com/jackfruittt/ROS-TCP-Connector.git
```

Verify `Packages/manifest.json` contains:
```json
"com.unity.robotics.ros-tcp-connector": "file:/home/<user>/Software/ROS-TCP-Connector/com.unity.robotics.ros-tcp-connector"
```

Update the path to match your username if different.

### URDF Importer

```bash
cd ~
git clone https://github.com/Unity-Technologies/URDF-Importer.git
```

Verify `Packages/manifest.json` contains:
```json
"com.unity.robotics.urdf-importer": "file:/home/<user>/URDF-Importer/com.unity.robotics.urdf-importer"
```

### AprilTag (Keijiro)

Installed automatically from the scoped NPM registry (`registry.npmjs.com`) declared in `Packages/manifest.json`. No manual step required -- Unity Package Manager fetches it on first open.

Package: `jp.keijiro.apriltag` v1.0.2

### All Other Packages

The following are installed from the Unity registry automatically:

| Package | Version |
|---|---|
| Universal Render Pipeline | 17.3.0 |
| Input System | 1.18.0 |
| XR Interaction Toolkit | 3.3.1 |
| XR Hands | 1.7.3 |
| Meta OpenXR | 2.4.0 |
| AR Foundation | 6.3.4 |

---

## 4. Update Local Package Paths

The manifest references absolute paths for the local packages. After cloning, update them to match your machine:

```bash
# In Packages/manifest.json, replace the two file:// paths:
"com.unity.robotics.ros-tcp-connector": "file:/home/YOUR_USER/Software/ROS-TCP-Connector/com.unity.robotics.ros-tcp-connector",
"com.unity.robotics.urdf-importer":     "file:/home/YOUR_USER/URDF-Importer/com.unity.robotics.urdf-importer",
"com.unity.robotics.visualizations":    "file:/home/YOUR_USER/Software/ROS-TCP-Connector/com.unity.robotics.visualizations"
```

---

## 5. Open in Unity

1. Open Unity Hub, click **Open**, select the `ur3_XR_sim` folder
2. Unity will import all packages on first open (may take a few minutes)
3. Open `Assets/Scenes/GetStarted_Scene`

---

## 6. Configure ROS Connection

1. In the Unity menu bar: `Robotics -> ROS Settings`
2. Set **ROS IP Address** to your ROS2 machine IP
3. Leave port at 10000 (default ROS-TCP endpoint port)

---

## 7. Running the System

### Simulation only (no physical robot or camera)

```bash
# Terminal 1: ROS-TCP endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# Terminal 2: UR3e MoveIt sim (optional, needed for /joint_states)
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e use_fake_hardware:=true
```

Then hit Play in Unity. Enable **Pseudo Detection** on `HandEyeCalibrationCollector` to test the calibration pipeline without a camera.

The endpoint uses a `MultiThreadedExecutor` with 4 threads by default, which is what allows 90Hz RealSense streams to render at 60Hz in Unity without frame stalls. Override with:
```bash
export ROS_TCP_EXECUTOR_THREADS=8  # increase if adding more high-rate topics
```

### With physical robot and D455

```bash
# Terminal 1: ROS-TCP endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# Terminal 2: RealSense streams
python3 d455_unity_streaming.launch.py

# Terminal 3: UR robot driver (update robot_ip)
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.1.100
```

---

## 8. Calibration JSON Output

The solved calibration is written to:

```
~/.config/unity3d/<CompanyName>/<ProductName>/hand_eye_calibration.json
```

Check `Application.persistentDataPath` in the Unity console on first run to confirm the exact path on your machine.
