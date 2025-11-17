from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_module',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[{'config_file': '/path/to/config/yolo.yaml'}],
            remappings=[('/input/image', '/camera/image_raw')]
        )
    ])