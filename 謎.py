import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml

# 固定パスで取得（環境変数に依存しない）
BASE_PATH = '/workspace/ros2_ws/src/crane_x7_ros'
MOVEIT_CONFIG_PATH = os.path.join(BASE_PATH, 'crane_x7_moveit_config')
CONTROL_CONFIG_PATH = os.path.join(BASE_PATH, 'crane_x7_control')
EXAMPLES_PATH = os.path.join(BASE_PATH, 'crane_x7_examples')

from crane_x7_description.robot_description_loader import RobotDescriptionLoader

def load_file(file_path):
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    declare_port_name = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyUSB0',
        description='Set port name.'
    )

    declare_baudrate = DeclareLaunchArgument(
        'baudrate',
        default_value='3000000',
        description='Set baudrate.'
    )

    declare_use_d435 = DeclareLaunchArgument(
        'use_d435',
        default_value='false',
        description='Use d435.'
    )

    # Robot Description
    config_file_path = os.path.join(CONTROL_CONFIG_PATH, 'config', 'manipulator_config.yaml')
    links_file_path = os.path.join(CONTROL_CONFIG_PATH, 'config', 'manipulator_links.csv')

    description_loader = RobotDescriptionLoader()
    description_loader.port_name = LaunchConfiguration('port_name')
    description_loader.baudrate = LaunchConfiguration('baudrate')
    description_loader.use_d435 = LaunchConfiguration('use_d435')
    description_loader.timeout_seconds = '1.0'
    description_loader.manipulator_config_file_path = config_file_path
    description_loader.manipulator_links_file_path = links_file_path

    description = description_loader.load()
    robot_description = {'robot_description': description}

    robot_description_semantic = {
        'robot_description_semantic': load_file(os.path.join(MOVEIT_CONFIG_PATH, 'config', 'crane_x7.srdf'))
    }

    robot_description_planning = {
        "robot_description_planning": load_yaml(os.path.join(MOVEIT_CONFIG_PATH, "config", "joint_limits.yaml"))
    }

    kinematics_yaml = load_yaml(os.path.join(MOVEIT_CONFIG_PATH, 'config', 'kinematics.yaml'))

    ompl_planning_pipeline_config = {'move_group': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization \
                             default_planner_request_adapters/FixWorkspaceBounds \
                             default_planner_request_adapters/FixStartStateBounds \
                             default_planner_request_adapters/FixStartStateCollision \
                             default_planner_request_adapters/FixStartStatePathConstraints',
        'start_state_max_bounds_error': 0.1
    }}
    ompl_planning_yaml = load_yaml(os.path.join(MOVEIT_CONFIG_PATH, 'config', 'ompl_planning.yaml'))
    if ompl_planning_yaml is not None:
        ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    moveit_controllers = {
        'moveit_simple_controller_manager': load_yaml(os.path.join(MOVEIT_CONFIG_PATH, 'config', 'controllers.yaml')),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.1
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True
    }

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[robot_description,
                    robot_description_semantic,
                    robot_description_planning,
                    kinematics_yaml,
                    ompl_planning_pipeline_config,
                    trajectory_execution,
                    moveit_controllers,
                    planning_scene_monitor_parameters]
    )

    rviz_config_file_default = os.path.join(MOVEIT_CONFIG_PATH, 'launch', 'run_move_group.rviz')
    rviz_config_file_camera = os.path.join(EXAMPLES_PATH, 'launch', 'camera_example.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file_default],
        parameters=[robot_description,
                    robot_description_semantic,
                    ompl_planning_pipeline_config,
                    kinematics_yaml],
        condition=UnlessCondition(LaunchConfiguration('use_d435'))
    )

    rviz_node_camera = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file_camera],
        parameters=[robot_description,
                    robot_description_semantic,
                    ompl_planning_pipeline_config,
                    kinematics_yaml],
        condition=IfCondition(LaunchConfiguration('use_d435'))
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link']
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    control_node = Node(
        package='crane_x7_control',
        executable='crane_x7_control_node',
        name='crane_x7_control',
        output='screen',
        parameters=[robot_description]
    )

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        output='screen',
        parameters=[{
            'camera_namespace': '',
            'device_type': 'd435',
            'pointcloud.enable': True,
            'align_depth.enable': True,
        }],
        condition=IfCondition(LaunchConfiguration('use_d435'))
    )

    servo_yaml = os.path.join(MOVEIT_CONFIG_PATH, 'config', 'servo_config.yaml')
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_server",
        parameters=[servo_yaml],
        output="screen"
    )

    return LaunchDescription([
        declare_port_name,
        declare_baudrate,
        declare_use_d435,
        run_move_group_node,
        rviz_node,
        rviz_node_camera,
        static_tf,
        robot_state_publisher,
        control_node,
        realsense_node,
        servo_node,
    ])
