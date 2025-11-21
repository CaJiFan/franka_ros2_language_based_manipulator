from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('vision_module'),
        'config',
        'yolo.yaml'
    ])

    return LaunchDescription([
        Node(
            package='vision_module',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[config_file],
            remappings=[('/input/image', '/camera/image_raw')]
        )
    ])