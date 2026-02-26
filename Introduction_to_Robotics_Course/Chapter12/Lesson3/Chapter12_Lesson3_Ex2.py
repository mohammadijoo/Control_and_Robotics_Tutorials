from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pub_node = Node(
        package='my_cpp_pkg',
        executable='simple_publisher',
        name='pub',
        namespace='demo',
        parameters=[{'rate_hz': 2.0}],
        remappings=[('chatter', 'demo_chatter')]
    )

    sub_node = Node(
        package='my_py_pkg',
        executable='simple_subscriber',
        name='sub',
        namespace='demo',
        remappings=[('chatter', 'demo_chatter')]
    )

    return LaunchDescription([pub_node, sub_node])
      
