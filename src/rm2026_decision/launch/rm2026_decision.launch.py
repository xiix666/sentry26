import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="rm2026_decision",
            executable="rm2026_decision_node",
            name="rm2026_decision_node",
            output="screen",
        ),
    ])
