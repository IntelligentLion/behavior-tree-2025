from setuptools import find_packages, setup
entry_points={
    'console_scripts': [
        'thruster_node = mavlink_thruster_node.thruster_node:main',
    ],
},



package_name = 'mavlink_thruster_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yirehban',
    maintainer_email='yirehban@todo.todo',
    description='Control Pixhawk thrusters via ROS 2 and pymavlink',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'thruster_node = mavlink_thruster_control.thruster_node:main'
        ],
    },
)
