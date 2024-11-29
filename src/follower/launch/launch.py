from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    follower_node = Node(
        package='follower',
        executable='follower',
        output='screen',
        shell=True
    )
    
    return LaunchDescription([
        follower_node
    ])