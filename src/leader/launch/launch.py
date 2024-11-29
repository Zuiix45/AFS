from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    xrce_dds_agent = ExecuteProcess(
        cmd=[[
            'MicroXRCEAgent udp4 --port 8888'
        ]],
        shell=True
    )

    pos_publisher_node = Node(
        package='leader',
        executable='pos_publisher',
        output='screen',
        shell=True
    )
    
    route_manager_node = Node(
        package='leader',
        executable='route_manager',
        output='screen',
        shell=True
    )

    return LaunchDescription([
        xrce_dds_agent,
        pos_publisher_node,
        route_manager_node
    ])