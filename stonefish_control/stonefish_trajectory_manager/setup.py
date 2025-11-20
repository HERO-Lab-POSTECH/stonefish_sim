from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'stonefish_trajectory_manager'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'examples'), glob('config/examples/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stonefish Control Team',
    maintainer_email='dev@example.com',
    description='Path generation and following for Stonefish UUV simulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'path_generator_node = stonefish_trajectory_manager.nodes.path_generator_node:main',
            'path_following_node = stonefish_trajectory_manager.nodes.path_following_node:main',
        ],
    },
)
