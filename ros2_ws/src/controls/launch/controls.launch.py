from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controls',
            executable='sim',
            name='sim'
        ),
        Node(
            package='controls',
            executable='traj',
            name='traj'
        ),
        # Node(
        #     package='controls',
        #     executable='fsm',
        #     name='fsm'
        # ),
        Node(
            package='controls',
            executable='legs',
            name='legs'
        )
        Node(
            package='controls',
            executable='movement_controller',
            name = 'movement_controller'
        ),
    ])