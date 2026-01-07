import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import yaml
import xacro

def load_yaml(package_name, file_path):
    package_path = FindPackageShare(package_name).find(package_name)
    file_path = os.path.join(package_path, file_path)
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # --- 1. ARGUMENTS ---
    args = [
        DeclareLaunchArgument('robot_ip', default_value='172.16.0.2'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('mission_file', default_value='/home/userlab/cjimenez/franka_ros2_language_based_manipulator/shared/sequence.json')
    ]

    robot_ip = LaunchConfiguration('robot_ip')
    use_fake = LaunchConfiguration('use_fake_hardware')
    mission_file = LaunchConfiguration('mission_file')

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

    # --- 2. PREPARE PARAMS FOR YOUR NODE ---
    moveit_config_pkg = 'franka_fr3_moveit_config'

    # KINEMATIC

    # --- 3. THE MAIN MOVEIT LAUNCH ---
    # This handles drivers, controllers, move_group, and RViz all in one go.
    
    franka_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(moveit_config_pkg), 'launch', 'moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake,
            'load_gripper': 'true',         # Try 'load_gripper' first
        }.items(),
    )

    # # --- 4. TF PUBLISHER (Camera) ---
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_broadcaster',
        arguments=[
            # Position (X, Y, Z)
            # '0.3966', '0.4126', '0.6594',
            # '0.38600361', '0.51878038', '0.68019778',
            '0.39098521', '0.48595019', '0.70823142',
            
            # Rotation (Qx, Qy, Qz, Qw)
            # '0.94210637', '-0.07166619', '-0.11180447',  '0.30789496',
            '-0.1052', '0.9499', '-0.2924', '0.0324',
            # '-0.5', '0.0', '1.0', '0.0',
            
            # Frame IDs
            'fr3_link0',      # Parent (Robot Base)
            'camera_link'     # Child (Camera)
        ],
        output='screen'
    )

    # --- 5. YOUR NODE ---
    move_node = Node(
        package='monitor_cpp',
        executable='move_to_pose', 
        name='move_to_pose_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            {'mission_file': mission_file}
        ]
    )

    return LaunchDescription(args + [
        franka_moveit_launch,
        static_tf,
        move_node
    ])