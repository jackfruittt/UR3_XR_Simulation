from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess
import os

def generate_launch_description():
    # Detect USB connection speed
    script_dir = os.path.dirname(os.path.abspath(__file__))
    usb_check_script = os.path.join(script_dir, 'check_realsense_usb.sh')
    
    usb_speed = "unknown"
    enable_imu = False
    
    try:
        result = subprocess.run([usb_check_script], capture_output=True, text=True, timeout=5)
        usb_speed = result.stdout.strip()
        
        if usb_speed in ["3.0", "3.1", "3.2"]:
            enable_imu = True
            print(f"[RealSense] Detected USB {usb_speed} connection - IMU/Gyro ENABLED")
        elif usb_speed == "2.0":
            enable_imu = False
            print(f"[RealSense] Detected USB 2.0 connection - IMU/Gyro DISABLED (prevents stalling)")
        else:
            enable_imu = False
            print(f"[RealSense] Could not detect USB speed - IMU/Gyro DISABLED (safe default)")
    except Exception as e:
        enable_imu = False
        print(f"[RealSense] Error detecting USB speed: {e} - IMU/Gyro DISABLED (safe default)")
    
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            parameters=[{
                # Enable streams - BASIC CONFIG
                'enable_depth': True,
                'enable_color': True,
                'enable_infra1': False,
                'enable_infra2': False,
                
                # MOTION MODULE - auto-enabled on USB 3.0, disabled on USB 2.0
                'enable_accel': enable_imu,
                'enable_gyro': enable_imu,
                
                # Simple resolution settings
                # D455 native valid modes: depth=848x480, 640x480 | color=1280x720, 640x480
                # Using 848x480 depth + 1280x720 color (align_depth maps depth -> color res)
                'depth_module.profile': '848x480x30',
                'rgb_camera.profile': '1280x720x30',
                
                # OUTPUT RAW 16-BIT DEPTH (not colorized)
                'depth_module.depth_format': 'Z16',  # 16-bit raw depth
                
                # DISABLE COLORIZER - must output raw 16-bit depth!
                'colorizer.enable': False,
                
                # Align depth to color frame
                'align_depth.enable': True,
                
                # DISABLE ALL FILTERS to test
                'spatial_filter.enable': False,
                'temporal_filter.enable': False,
                'hole_filling_filter.enable': False,
                'disparity_filter.enable': False,
                
                # Keep emitter on for depth
                'depth_module.emitter_enabled': 1,
            }],
            output='screen',
        )
    ])
