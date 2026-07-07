from setuptools import setup

package_name = 'g1_zed_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/zed_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex Chiu',
    maintainer_email='alexchiu0108@gmail.com',
    description='Decodes the G1 ZED Mini stream into ROS 2 Image/PointCloud2 topics',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'zed_stream_node = g1_zed_bridge.zed_stream_node:main',
        ],
    },
)
