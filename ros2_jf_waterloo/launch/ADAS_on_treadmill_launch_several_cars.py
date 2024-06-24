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
            executable='avoiding_obstacles_node',
            name='avoiding_obstacles_node',
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
        # Node(
        #     package='jf_package_waterloo',
        #     executable='treadmill_control_node',
        #     name='treadmill_control_node',
        # ),
        # Node(
        #     package='jf_package_waterloo',
        #     executable='security_node',
        #     name='security_node',
        # )
    ])