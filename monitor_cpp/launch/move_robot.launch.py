from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import yaml

def generate_launch_description():
    # --- 1. Declare Arguments (Flags) ---
    # These let you type "x:=0.5" in the terminal
    args = [
        DeclareLaunchArgument('x', default_value='0.5', description='Target X'),
        DeclareLaunchArgument('y', default_value='0.0', description='Target Y'),
        DeclareLaunchArgument('z', default_value='0.4', description='Target Z'),
        DeclareLaunchArgument('r', default_value='180.0', description='Roll (deg)'),
        DeclareLaunchArgument('p', default_value='0.0', description='Pitch (deg)'),
        DeclareLaunchArgument('yw', default_value='45.0', description='Yaw (deg)'),
    ]

    # --- 2. Load Robot Configs ---
    franka_desc_pkg = get_package_share_directory('franka_description')
    xacro_file = os.path.join(franka_desc_pkg, 'robots', 'fr3', 'fr3.urdf.xacro')
    moveit_config_pkg = get_package_share_directory('franka_fr3_moveit_config')
    srdf_file = os.path.join(moveit_config_pkg, 'config', 'fr3.srdf')
    kinematics_file = os.path.join(moveit_config_pkg, 'config', 'kinematics.yaml')

    doc = xacro.process_file(xacro_file, mappings={'hand': 'true'})
    robot_description = {'robot_description': doc.toxml()}

    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    with open(kinematics_file, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    # --- 3. Define Node ---
    move_node = Node(
        package='monitor_cpp',
        executable='move_to_pose',
        name='move_to_pose_node',
        namespace='NS_1', # Check your namespace!
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            # Pass the launch arguments into the node
            {
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'r': LaunchConfiguration('r'),
                'p': LaunchConfiguration('p'),
                'yw': LaunchConfiguration('yw'),
            }
        ]
    )

    return LaunchDescription(args + [move_node])