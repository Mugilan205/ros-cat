from setuptools import setup, find_packages

package_name = 'your_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='caterpillar',
    maintainer_email='caterpillar@todo.todo',
    description='Bringup package for robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_odom = your_robot_bringup.wheel_odom:main',
            'rrt_autonomous = your_robot_bringup.rrt_autonomous:main',

        ],
    },
)

