from setuptools import setup, find_packages

package_name = 'rover_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required by ament
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),

        # Launch files
        ('share/' + package_name + '/launch', [
            'launch/obstacle_detection.launch.py',
            'launch/rrt_planner.launch.py',
            'launch/full_navigation.launch.py',
            'launch/a_star_lwb_navigation.launch.py',
        ]),

        # Nav2 configuration
        ('share/' + package_name + '/config', [
            'config/nav2_params.yaml',
            'config/slam_params.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Rover Developer',
    author_email='rover@example.com',
    maintainer='Rover Developer',
    maintainer_email='rover@example.com',
    description='Rover navigation stack with Nav2 integration',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'obstacle_detection = rover_navigation.obstacle_detection:main',
            'rrt_planner = rover_navigation.rrt_planner:main',
            'path_executor = rover_navigation.path_executor:main',
            'odometry_node = rover_navigation.odometry_node:main',
            'esp8266_bridge = rover_navigation.esp8266_bridge:main',
            'a_star_planner = rover_navigation.global_planner_a_star:main',
            'lwb_planner = rover_navigation.local_planner_lwb:main',
            'planner_manager = rover_navigation.planner_manager:main',
            'navigation_dashboard = rover_navigation.dashboard_node:main',
            'esp_serial_odometry_node = rover_navigation.esp_serial_odometry_node:main',
            'cmdvel_to_wheels = rover_navigation.cmdvel_to_wheels:main',
        ],
    },
)