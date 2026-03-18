from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            parameters=[{
                # Enable both streams
                'enable_depth': True,
                'enable_color': True,
                'enable_infra1': False,
                'enable_infra2': False,
                
                # Resolution and frame rate (D455 native depth resolution is 848x480)
                'depth_module.profile': '848x480x30',
                'rgb_camera.profile': '640x480x30',
                
                # Laser emitter settings (CRITICAL for depth quality)
                'depth_module.emitter_enabled': 1,
                'laser_power': 150,
                
                # Depth processing parameters from RealSense Viewer JSON
                'depth_module.gain': 16,
                
                # Advanced stereo matching parameters
                'stereo_module.lambda_ad': 800,
                'stereo_module.lambda_census': 26,
                'stereo_module.left_right_threshold': 24,
                'stereo_module.median_threshold': 500,
                'stereo_module.neighbor_thresh': 7,
                
                # Align depth to color frame
                'align_depth.enable': True,
                
                # Colorizer - rainbow/jet colormap like RealSense Viewer
                'enable_color_depth': True,
                'colorizer.enable': True,
                'colorizer.color_scheme': 0,  # 0=Jet (rainbow), 2=WhiteToBlack, 3=BlackToWhite
                
                # Post-processing filters
                'spatial_filter.enable': True,
                'spatial_filter.magnitude': 2,
                'spatial_filter.smooth_alpha': 0.5,
                'spatial_filter.smooth_delta': 20,
                'spatial_filter.holes_fill': 0,
                
                'temporal_filter.enable': True,
                'temporal_filter.smooth_alpha': 0.4,
                'temporal_filter.smooth_delta': 20,
                'temporal_filter.persistence_control': 3,
                
                'hole_filling_filter.enable': False,
                'hole_filling_filter.mode': 1,  # farthest_from_around
                
                # Disparity filter disabled - causes DISPARITY32 format error
                # Spatial/temporal filters work fine directly on depth
                'disparity_filter.enable': False,
            }],
            output='screen',
        )
    ])
