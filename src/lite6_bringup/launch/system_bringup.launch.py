import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    desc_pkg = get_package_share_directory('lite6_description')
    hw_pkg = get_package_share_directory('lite6_hardware')

    urdf_file = os.path.join(desc_pkg, 'urdf', 'lite6.urdf')
    with open(urdf_file, 'r') as infp: 
        robot_desc = infp.read()
    
    controller_config = os.path.join(hw_pkg, 'config', 'ros2_controllers.yaml')

    # Core nodes for the robot system bringup
    control_node = Node(
        package="controller_manager", 
        executable="ros2_control_node", 
        parameters=[{'robot_description': robot_desc}, controller_config], 
        output="screen"
    )
    
    rsp_node = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher', 
        parameters=[{'robot_description': robot_desc}, {'publish_frequency': 200.0}]
    )

    # Native MuJoCo display in its own process. The hardware physics thread is
    # independent of window refresh, resizing, minimization and window closure.
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

    # Load controllers: Joint State Broadcaster and CTC Controller
    jsb_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    ctc_spawner = Node(package="controller_manager", executable="spawner", arguments=["ctc_controller"])
    # Planner and HMI nodes
    planner_node = Node(
        package='lite6_planner', 
        executable='lite6_planner_node', 
        name='lite6_industrial_planner',
        output='screen'
    )

    hmi_node = Node(
        package='lite6_hmi', 
        executable='gui_main', 
        output='screen'
    )

    delay_upper_layer_after_ctc = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=ctc_spawner, on_exit=[planner_node, hmi_node])
    )

    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='true', description='Open native MuJoCo window'),
        DeclareLaunchArgument('viewer_fps', default_value='60', description='Render rate, 15..240 Hz'),
        DeclareLaunchArgument('viewer_shadows', default_value='false', description='Render cast shadows (may show mesh artifacts)'),
        DeclareLaunchArgument('viewer_shadow', default_value='false', description='Show cyan target robot'),
        control_node, rsp_node, viewer_node, jsb_spawner, ctc_spawner,
        # 2. Launch the planner and HMI after the direct trajectory CTC controller is ready.
        delay_upper_layer_after_ctc
    ])
