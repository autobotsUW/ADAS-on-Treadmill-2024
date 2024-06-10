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
            executable='input_node',
            name='input_node',
        ),
        Node(
            package='jf_package_waterloo',
            executable='control_node',
            name='control_node',
        ),
        Node(
            package='jf_package_waterloo',
            executable='camera_node',
            name='camera_node',
        ),
        Node(
            package='jf_package_waterloo',
            executable='display_node',
            name='display_node',
        ),
        Node(
            package='rqt_graph',
            executable='rqt_graph',
            name='rqt_graph',
        ),
        Node(
            package='jf_package_waterloo',
            executable='treadmill_control_node',
            name='treadmill_control_node',
        ),
        Node(
            package='jf_package_waterloo',
            executable='security_node',
            name='security_node',
        )
    ])