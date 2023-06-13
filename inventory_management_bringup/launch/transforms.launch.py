from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    static_broadcast_node = Node(
        package = "inventory_manager_bot",
        executable = "static_broadcaster"
    )
    odom_broadcast_node = Node(
        package = "inventory_manager_bot",
        executable = "odom_broadcaster"
    )
    map_transform_node = Node(
        package = "inventory_manager_bot",
        executable = "map_transform"
    )
    
    ld.add_action(static_broadcast_node)
    ld.add_action(odom_broadcast_node)
    ld.add_action(map_transform_node)
    
    return ld