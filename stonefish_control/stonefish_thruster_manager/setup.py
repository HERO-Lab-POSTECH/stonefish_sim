from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'stonefish_thruster_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seungmin Kim',
    maintainer_email='luckkim123@gmail.com',
    description='Thruster allocation manager for Stonefish simulator',
    license='GPL-3.0',
    entry_points={
        'console_scripts': [
            'thruster_allocator = stonefish_thruster_manager.nodes.thruster_allocator_node:main',
        ],
    },
)
