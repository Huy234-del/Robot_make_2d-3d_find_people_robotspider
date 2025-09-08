from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rescue_robot',
            executable='rescue_robot_node',
            name='rescue_robot',
            output='screen',
            parameters=[
                {'confidence_threshold': 0.6},
                {'thermal_threshold': 35.0},
                {'grid_resolution': 0.1},
                {'grid_width': 200},
                {'grid_height': 200}
            ]
        ),
    ])