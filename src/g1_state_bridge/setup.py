from setuptools import setup

package_name = 'g1_state_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/real_state.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex Chiu',
    maintainer_email='alexchiu0108@gmail.com',
    description='Bridges Unitree G1 LowState to sensor_msgs/JointState',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'state_bridge_node = g1_state_bridge.state_bridge_node:main',
            'hand_controller_gui = g1_state_bridge.hand_controller_gui:main',
            'neck_controller_gui = g1_state_bridge.neck_controller_gui:main',
        ],
    },
)
