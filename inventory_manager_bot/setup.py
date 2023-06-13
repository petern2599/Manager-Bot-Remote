from setuptools import setup

package_name = 'inventory_manager_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='petern25',
    maintainer_email='petern25@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "robot_caller_server = inventory_manager_bot.robot_caller_server:main",
            "robot_caller_client = inventory_manager_bot.robot_caller_interface:main",
            "keyboard_teleop = inventory_manager_bot.keyboard_teleop:main",
            "serial_logger = inventory_manager_bot.serial_logger:main",
            "label_reader = inventory_manager_bot.label_reader:main",
            "inventory_validator = inventory_manager_bot.inventory_validator:main",
            "static_broadcaster = inventory_manager_bot.static_broadcaster:main",
            "odom_broadcaster = inventory_manager_bot.transform_broadcaster:main",
            "map_transform = inventory_manager_bot.map_transform_listener:main",
            "map_recorder = inventory_manager_bot.map_recorder:main",
            "path_planner_server = inventory_manager_bot.path_planner_server:main",
            "control_logger = inventory_manager_bot.controller_logger:main",
            "waypoint_publisher = inventory_manager_bot.waypoint_publisher:main"
        ],
    },
)
