from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'stonefish_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'bluerov2'), glob('config/bluerov2/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Unified control package for Stonefish simulator',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hybrid_controller_node = stonefish_control.controllers.hybrid_controller_node:main',
            'position_controller_node = stonefish_control.nodes.position_controller_node:main',
            'velocity_controller_node = stonefish_control.controllers.velocity_controller_node:main',
        ],
    },
)
