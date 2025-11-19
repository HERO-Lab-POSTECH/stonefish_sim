from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'stonefish_control_utils'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config', 'optimizer'),
            glob('config/optimizer/*.yaml')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Install wrapper scripts to lib directory for ROS2 launch compatibility
        (os.path.join('lib', package_name),
            glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Utility tools for PID tuning and optimization in Stonefish simulator',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pid_tuning_tool = stonefish_control_utils.pid_tools.tuning_tool_node:main',
            'pid_optimizer = stonefish_control_utils.pid_optimizer.node:main',
        ],
    },
)
