import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate the RealSense package
    rs_pkg_path = get_package_share_directory('realsense2_camera')
    
    # 2. Define the RealSense Launch (forcing align_depth=true)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rs_pkg_path, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'align_depth.enable': 'true',  # Critical for your 3D calculations
            'pointcloud.enable': 'false'   # Disable if you don't need raw PC (saves CPU)
        }.items()
    )

    # 3. Define Your Vision Node
    vision_node = Node(
        package='vision_module',
        executable='vision_node',
        name='vision_node',
        output='screen'
    )

    return LaunchDescription([
        realsense_launch,
        vision_node
    ])