from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jf_package_waterloo',
            executable='bluetooth_node',
            name='bluetooth_node',
        ),
        Node(
            package='jf_package_waterloo',
            executable='car_on_road_input_node',
            name='car_on_road_input_node',
        ),
        Node(
            package='jf_package_waterloo',
            executable='control_node',
            name='control_node',
        ),
        Node(
            package='jf_package_waterloo',
            executable='camera_with_apriltags_node',
            name='camera_with_apriltags_node',
        ),
        Node(
            package='jf_package_waterloo',
            executable='display_node',
            name='display_node',
        ),
        Node(
            package='jf_package_waterloo',
            executable='save_topic',
            name='save_topic',
        ),
        Node(
            package='jf_package_waterloo',
            executable='trust_score_node',
            name='trust_score_node',
        ),
    ])