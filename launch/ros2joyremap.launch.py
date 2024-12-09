import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
import ament_index_python.packages

def generate_launch_description():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('ros2joyremap'))
    params = os.path.join(config_directory, 'ros2joyremap.params.yaml')

    joy_node = launch_ros.actions.Node(
            package='joy', 
            node_namespace="/ambotw1_ns",
            node_executable='joy_node', 
            output='screen',
            node_name="joy_node")

    joyremap_node = launch_ros.actions.Node(
            package='ros2joyremap', 
            node_executable='ros2joyremap_node',
            node_namespace="/ambotw1_ns",
            output='screen',
            node_name="joyremap_node",
            parameters=[params],
            )

    return launch.LaunchDescription(
            [
            joy_node,
            joyremap_node,
            launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=joyremap_node,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
                )),
            ])
