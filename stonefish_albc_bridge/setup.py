import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'stonefish_albc_bridge'

setup(
    name=package_name,
    version='0.5.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'policy'),
            glob(os.path.join(package_name, 'policy', '*.py')) +
            glob(os.path.join(package_name, 'policy', '*.npz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HERO Lab, POSTECH',
    maintainer_email='luckkim123@gmail.com',
    description='RL policy bridge: obs-policy-action loop between Stonefish and a trained ALBC policy',
    license='GPL-3.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stonefish_albc_bridge = stonefish_albc_bridge.bridge_node:main',
        ],
    },
)
