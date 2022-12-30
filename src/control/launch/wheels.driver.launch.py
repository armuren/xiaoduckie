from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "xiaoduckie", package='control', executable='wheels_driver_node', output='screen'),
    ])
