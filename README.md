# UR3 XR Simulation

A Unity-based XR simulation environment for controlling and visualizing a Universal Robots UR3 collaborative robot with real-time ROS2 integration and Intel RealSense camera streaming.

## Overview

This project provides an immersive XR interface for robotic manipulation, enabling users to control a UR3 robot arm through intuitive UI controls while visualizing real-time camera feeds from Intel RealSense depth cameras. The system bridges Unity's physics simulation with ROS2, allowing seamless integration with real robotic systems.

## Features

- **Robot Control**: Interactive joint-level control of UR3 robot arm with real-time feedback
- **ROS2 Integration**: Bidirectional communication with ROS2 for joint states and commands
- **Camera Streaming**: Real-time visualization of RealSense D455 color and depth streams
- **Point Cloud Rendering**: Dynamic point cloud visualization from depth camera data
- **XR Support**: VR/AR ready interface for immersive robot interaction
- **Orbit Camera**: Smooth camera controls for scene navigation and inspection

## Requirements

### Software
- Unity 2022.3 LTS or later
- ROS2 (Humble or later)
- Unity Robotics Hub packages
- Intel RealSense SDK (for hardware streaming)

### Hardware (Optional)
- VR/XR headset (Quest, Vive, etc.)
- Intel RealSense D455 depth camera
- UR3/UR3e robot arm (for physical deployment)

## Project Structure

```
Assets/
├── Scenes/              # Unity scenes
├── Scripts/
│   ├── ROS/            # ROS communication scripts
│   │   ├── ROSPointCloudRenderer.cs
│   │   ├── SimpleImageSubscriber.cs
│   │   ├── SimpleJointController.cs
│   │   └── UR3SourceDestinationPublisher.cs
│   └── Unity_Control/  # Unity-specific controls
│       └── OrbitCamera.cs
├── Prefabs/            # Reusable game objects
├── Materials/          # Visual materials
└── ur_e_description/   # Robot URDF files
```

## Setup

### 1. Unity Configuration
1. Open project in Unity 2022.3 LTS or later
2. Install Unity Robotics Hub from Package Manager
3. Configure ROS-TCP-Connector settings (typically `localhost:10000`)

### 2. ROS2 Setup
```bash
# Launch ROS2 TCP endpoint
ros2 run ros_tcp_endpoint default_server_endpoint

# (Optional) Launch RealSense camera streaming
python3 d455_unity_streaming.launch.py
```

### 3. Scene Configuration
1. Open main scene from `Assets/Scenes/`
2. Verify robot prefab is loaded with ArticulationBody components
3. Assign UI elements to `SimpleJointController` in Inspector
4. Configure camera topics in `SimpleImageSubscriber` components

## Usage

### Robot Control
- Use sliders or input fields to control individual joint angles
- Click **Publish** to send commands to ROS2
- Click **Home** to return robot to safe starting position
- Real-time joint state updates from ROS2 reflected in UI

### Camera Navigation
- **Left Mouse Drag**: Rotate camera around target
- **Right Mouse Drag**: Pan camera view
- **Mouse Wheel**: Zoom in/out
- **WASD**: Move target point
- **Q/E**: Vertical movement
- **F**: Focus on origin
- **R**: Reset view

### Camera Streaming
Configure camera topics in Inspector:
- Color camera: `/camera/camera/color/image_raw`
- Depth camera: `/camera/camera/depth/image_rect_raw`
- Aligned depth: `/camera/camera/aligned_depth_to_color/image_raw`

## Scripts Documentation

### ROS Scripts
- **ROSPointCloudRenderer.cs**: Renders 3D point clouds from ROS PointCloud2 messages
- **SimpleImageSubscriber.cs**: Subscribes to ROS image topics (raw/compressed, color/depth)
- **SimpleJointController.cs**: UI controller for robot joint manipulation
- **UR3SourceDestinationPublisher.cs**: Manages robot joint states and ROS communication

### Unity Control Scripts
- **OrbitCamera.cs**: Professional orbit camera controller with smooth damping

## Configuration Files

- `d455_unity_streaming.launch.py`: Launch file for RealSense streaming
- `realsense_settings_1.json`: Camera configuration presets
- `check_realsense_usb.sh`: USB connectivity diagnostic tool

## Troubleshooting

### ROS Connection Issues
- Verify ROS TCP endpoint is running on specified port
- Check firewall settings allow TCP connections
- Confirm `ROS_DOMAIN_ID` matches across systems

### Camera Streaming
- Ensure RealSense SDK is properly installed
- Check USB 3.0 connection with `check_realsense_usb.sh`
- Verify camera topics are publishing: `ros2 topic list`

### Robot Control
- Confirm ArticulationBody components on robot joints
- Verify joint limits are configured correctly in Unity
- Check joint names match between Unity and ROS

## Development

### Adding New Features
1. Create new scripts in appropriate subdirectory
2. Follow existing naming conventions and documentation style
3. Use Unity's ArticulationBody for robot joints
4. Leverage ROSConnection singleton for ROS communication

### Testing
- Test scripts available in `Assets/Scripts/ROS/` for debugging
- Use Unity Console for real-time logging
- Monitor ROS topics with `ros2 topic echo`

## License



## Contributors

Jackson Russell

## Acknowledgments

- Unity Robotics Hub
- Universal Robots
- Intel RealSense SDK
- ROS2 Community
