from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'rover_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam_pipeline.launch.py']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ezhilan404',
    maintainer_email='ezhilan.404@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_bridge = rover_control.cmd_bridge:main',
            'risk_heatmap = rover_control.risk_heatmap:main',
            'detection_node = rover_control.detection_node:main',
	    'astar_planner = rover_control.astar_planner:main',
        ],
    },
)
