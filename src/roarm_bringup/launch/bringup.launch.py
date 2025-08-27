import os
import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def _wait_for_compute_ik_and_start(context, *args, **kwargs):
    try:
        import rclpy
    except Exception as e:
        print('[bringup] rclpy not available in launch environment:', e)
        ik = Node(package='ik_solver', executable='ik_solver', output='screen')
        sirius = Node(package='sirius_reader', executable='sirius_reader', output='screen')
        return [ik, sirius]

    rclpy.init(args=None)
    node = rclpy.create_node('bringup_wait_for_compute_ik')
    timeout = 30.0
    start = time.time()
    try:
        while rclpy.ok():
            services = [name for (name, types) in node.get_service_names_and_types()]
            if '/compute_ik' in services:
                node.get_logger().info('/compute_ik service detected.')
                break
            if time.time() - start > timeout:
                node.get_logger().warn('Timeout waiting for /compute_ik; starting anyway.')
                break
            time.sleep(0.5)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()

    ik = Node(package='ik_solver', executable='ik_solver', output='screen')
    sirius = Node(package='sirius_reader', executable='sirius_reader', output='screen')
    return [ik, sirius]

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    use_driver = LaunchConfiguration('use_driver')

    args = [
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB1',
                              description='Serial port for roarm_driver'),
        DeclareLaunchArgument('baud_rate', default_value='115200',
                              description='Baud rate for roarm_driver'),
        DeclareLaunchArgument('use_driver', default_value='true',
                              description='Whether to start roarm_driver.'),
    ]

    robot_desc = ''
    rviz_config = ''
    try:
        share = get_package_share_directory('roarm_description')
        urdf_path = os.path.join(share, 'urdf', 'roarm_description.urdf')
        rviz_config = os.path.join(share, 'config', 'roarm_description.rviz')
        if os.path.exists(urdf_path):
            with open(urdf_path, 'r') as f:
                robot_desc = f.read()
    except Exception as e:
        print('[bringup] Warning: could not read roarm_description files:', e)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    rviz_args = ['-d', rviz_config] if rviz_config and os.path.exists(rviz_config) else []
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args,
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('roarm_moveit'), 'launch', 'move_group.launch.py')
        )
    )

    wait_and_start = OpaqueFunction(function=_wait_for_compute_ik_and_start)

    driver = Node(
        package='roarm_driver',
        executable='roarm_driver',
        output='screen',
        parameters=[{'serial_port': serial_port}, {'baud_rate': baud_rate}],
    )

    actions = []
    actions += args
    actions += [rsp, rviz]
    actions += [TimerAction(period=1.0, actions=[move_group_launch])]
    actions += [TimerAction(period=2.0, actions=[wait_and_start])]
    actions += [TimerAction(period=4.0, actions=[driver], condition=IfCondition(use_driver))]

    return LaunchDescription(actions)
