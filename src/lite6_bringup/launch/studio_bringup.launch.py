"""Standalone bringup launch file for the Lite6 Industrial Trajectory Studio stack."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_pkg = get_package_share_directory('lite6_description')
    hw_pkg = get_package_share_directory('lite6_hardware')

    urdf_file = os.path.join(desc_pkg, 'urdf', 'lite6.urdf')
    with open(urdf_file, 'r', encoding='utf-8') as infp:
        robot_desc = infp.read()

    controller_config = os.path.join(hw_pkg, 'config', 'ros2_controllers.yaml')
    compliance_controller_config = os.path.join(
        hw_pkg, 'config', 'controllers_compliance_mode.yaml'
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_desc},
            controller_config,
            compliance_controller_config,
        ],
        output='screen',
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}, {'publish_frequency': 200.0}],
    )

    viewer_node = Node(
        package='lite6_hardware',
        executable='mujoco_viewer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('viewer')),
        parameters=[{
            'fps': ParameterValue(LaunchConfiguration('viewer_fps'), value_type=int),
            'shadows': ParameterValue(LaunchConfiguration('viewer_shadows'), value_type=bool),
            'shadow': ParameterValue(LaunchConfiguration('viewer_shadow'), value_type=bool),
        }],
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    ctc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ctc_controller'],
    )

    studio_planner_node = Node(
        package='lite6_studio_planner',
        executable='lite6_studio_planner_node',
        name='lite6_studio_planner_node',
        output='screen',
    )

    studio_hmi_node = Node(
        package='lite6_studio_hmi',
        executable='studio_gui',
        name='lite6_studio_hmi_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    delay_studio_after_ctc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ctc_spawner,
            on_exit=[studio_planner_node, studio_hmi_node],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'viewer', default_value='true', description='Open native MuJoCo window'
        ),
        DeclareLaunchArgument(
            'gui', default_value='true', description='Open Lite6 Studio HMI PyQt5 window'
        ),
        DeclareLaunchArgument(
            'viewer_fps', default_value='60', description='Render rate, 15..240 Hz'
        ),
        DeclareLaunchArgument(
            'viewer_shadows', default_value='false', description='Render cast shadows'
        ),
        DeclareLaunchArgument(
            'viewer_shadow', default_value='false', description='Show cyan target robot'
        ),
        control_node,
        rsp_node,
        viewer_node,
        jsb_spawner,
        ctc_spawner,
        delay_studio_after_ctc,
    ])
