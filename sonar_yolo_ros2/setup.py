# SPDX-FileCopyrightText: 2025 Minjong Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later

from setuptools import find_packages, setup

package_name = 'sonar_yolo_ros2'

setup(
    name=package_name,
    version='0.5.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seungmin Kim',
    maintainer_email='luckkim123@gmail.com',
    description='YOLO object detection on FLS sonar images',
    license='GPL-3.0-or-later',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sonar_yolo_node = sonar_yolo_ros2.sonar_yolo_node:main',
        ],
    },
)
