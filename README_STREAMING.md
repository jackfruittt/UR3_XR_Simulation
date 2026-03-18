# Simple RealSense Image Streaming

## Quick Setup

### Unity
1. Create a Quad: GameObject > 3D Object > Quad
2. **Rotate the Quad** to face your camera (e.g., Rotation: 0, 180, 0)
3. Create empty GameObject, add `SimpleImageSubscriber` script
4. Drag Quad into the `Target Renderer` field
5. For compressed RGB: Check `Use Compressed` (saves bandwidth!)
6. Press Play

**Tip:** If the image disappears at sharp angles, the Quad's backface is showing. Either:
- Rotate the Quad 180° on Y-axis
- Or use a double-sided shader on the material

## ROS Setup - Compressed Images

### Install image transport plugins (if not already installed):
```bash
# ROS2 Humble/Iron/Jazzy
sudo apt install ros-$ROS_DISTRO-image-transport-plugins

# Or build from source if needed
cd ~/ros2_ws/src
git clone https://github.com/ros-perception/image_transport_plugins.git
cd ~/ros2_ws
colcon build --packages-select compressed_image_transport
source install/setup.bash
```

### Launch RealSense:

**Option 1: Optimized Launch (RECOMMENDED - matches RealSense Viewer settings)**
```bash
# From rs2_project directory
ros2 launch d455_unity_streaming.launch.py

# Or specify camera serial number if you have multiple cameras:
ros2 launch d455_unity_streaming.launch.py serial_no:='<your_serial_number>'
```

This launch file enables:
- ✅ Laser emitter ON (depth_module.emitter_enabled: 1)
- ✅ **848x480 @ 30fps depth** (D455 native resolution - CRITICAL!)
- ✅ 640x480 @ 30fps color
- ✅ Aligned depth to color camera
- ✅ Hole filling filter (fixes split arm artifacts)
- ✅ Spatial & temporal filters (smooth depth, reduced noise)
- ✅ Advanced stereo parameters from RealSense Viewer
- ✅ Matches settings that work well in RealSense Viewer

**Option 2: Default Launch (basic)**
```bash
ros2 launch realsense2_camera rs_launch.py
```

**Check it's working:**
```bash
# Verify emitter is enabled
ros2 param get /camera/realsense2_camera_node depth_module.emitter_enabled
# Should return: Integer value is: 1

# Check filters are active
ros2 param get /camera/realsense2_camera_node hole_filling_filter.enable
# Should return: Boolean value is: True
```

The `/compressed` topics should automatically be available:
```bash
# Check available topics
ros2 topic list | grep compressed

# Should see:
# /camera/camera/color/image_raw/compressed
# /camera/camera/depth/image_rect_raw/compressed (don't use this for depth!)
```

## Bandwidth Comparison

**RGB Color @ 1280x720 @ 30 FPS:**
- Raw: ~66 MB/s (2.2 MB per frame)
- Compressed (JPEG): ~3-6 MB/s (100-200 KB per frame)
- **Bandwidth savings: 90-95%!**

**Depth @ 640x480 @ 30 FPS:**
- Raw: ~18 MB/s
- Compressed: Not recommended (16-bit data doesn't compress well with JPEG)

## Unity Settings

**For RGB Color (RECOMMENDED - use compressed):**
- Use Depth Camera: ✗ Unchecked
- Use Aligned Depth: ✗ Unchecked
- Topic: `/camera/camera/color/image_raw`
- Use Compressed: ✓ Checked
- Flip Vertically: ✗ Unchecked (compressed images already correct)

**For Depth:**
- Use Depth Camera: ✓ Checked (topic: `/camera/camera/depth/image_rect_raw`)
- Use Aligned Depth: ✗ Unchecked
- Use Compressed: Disabled (depth uses 16-bit raw)
- Flip Vertically: ✓ Checked
- Max Depth MM: 5000 (adjust to visualize closer/farther objects)

**For Aligned Depth (depth aligned to color camera FOV):**
- Use Depth Camera: ✗ Unchecked
- Use Aligned Depth: ✓ Checked (topic: `/camera/camera/aligned_depth_to_color/image_raw`)
- Use Compressed: Disabled (depth uses 16-bit raw)
- Flip Vertically: ✓ Checked
- Max Depth MM: 5000 (adjust to visualize closer/farther objects)

## Script Features
- Auto-appends `/compressed` to topic if `Use Compressed` is checked
- Auto-detects depth (16UC1) vs color (rgb8/bgr8)
- Unity automatically decompresses JPEG/PNG
- FPS overlay

## Troubleshooting Split Arm in Depth

If your arm appears split or has holes in the depth image:

**⚠️ MOST COMMON ISSUE: Wrong Resolution**
The D455 camera's **native depth resolution is 848x480**. Using 640x480 causes interpolation artifacts (split arms, holes). Always use the native resolution for depth!

1. **Use correct resolution (848x480 for depth):**
   ```bash
   # Check current depth resolution:
   ros2 topic info /camera/camera/depth/image_rect_raw
   
   # Should be 848x480 for D455
   # If using the optimized launch file, this is already correct!
   ```

2. **Check emitter is ON:**
   ```bash
   ros2 param get /camera/realsense2_camera_node depth_module.emitter_enabled
   # Should be: Integer value is: 1
   ```

3. **Verify filters are enabled:**
   ```bash
   ros2 param get /camera/realsense2_camera_node hole_filling_filter.enable
   ros2 param get /camera/realsense2_camera_node spatial_filter.enable
   # Both should be: Boolean value is: True
   ```

4. **Check you're using aligned depth:**
   - Topic: `/camera/camera/aligned_depth_to_color/image_raw` ✓
   - NOT: `/camera/camera/depth/image_rect_raw` ✗

5. **If still seeing issues, try adjusting laser power:**
   ```bash
   # While camera is running, increase laser power for difficult scenes
   ros2 param set /camera/realsense2_camera_node depth_module.laser_power 250
   # Range: 0-360 (higher = better for low-texture surfaces like skin)
   ```

6. **Try different hole filling modes:**
   ```bash
   # Mode 1: farthest from around (best for arms/humans)
   ros2 param set /camera/realsense2_camera_node hole_filling_filter.mode 1
   
   # Mode 2: nearest from around (better for flat surfaces)
   ros2 param set /camera/realsense2_camera_node hole_filling_filter.mode 2
   ```

---

*Use compressed for RGB to save 90% bandwidth!*
