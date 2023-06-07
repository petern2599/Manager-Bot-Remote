from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    robot_caller_server_node = Node(
        package = "inventory_manager_bot",
        executable = "robot_caller_server",
        parameters=[
            {"robots":1},
            {"waiting_areas":1},
            {"user":"postgres"},
            {"password":"password"},
            {"host":"INSERT"},
            {"port":5432},
            {"db":"inventory"}
        ]
    )
    inventory_validator_node = Node(
        package = "inventory_manager_bot",
        executable = "inventory_validator",
        parameters=[
            {"user":"postgres"},
            {"password":"password"},
            {"host":"INSERT"},
            {"port":5432},
            {"db":"inventory"}
        ]
    )
    
    ld.add_action(inventory_validator_node)
    ld.add_action(robot_caller_server_node)
    
    return ld