import launch
import launch_ros


def generate_launch_description():
    action_node_ik_solver = launch_ros.actions.Node(
        package='ik_solver',
        executable='ik_solver',
        output='both',
    )

    action_node_roarm_driver = launch_ros.actions.Node(
        package='roarm_driver',
        executable='roarm_driver',
        output='both',
    )

    action_node_sirius_reader = launch_ros.actions.Node(
        package='sirius_reader',
        executable='sirius_reader',
        output='both',
    )

    launch_description = launch.LaunchDescription([
        action_node_ik_solver,
        action_node_roarm_driver,
        action_node_sirius_reader,
    ])

    return launch_description