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
            "image_publisher = inventory_manager_bot.image_publisher:main",
            "image_subscriber = inventory_manager_bot.image_subscriber:main",
            "robot_caller_server = inventory_manager_bot.robot_caller_server:main",
            "robot_caller_client = inventory_manager_bot.robot_caller_interface:main"
        ],
    },
)
