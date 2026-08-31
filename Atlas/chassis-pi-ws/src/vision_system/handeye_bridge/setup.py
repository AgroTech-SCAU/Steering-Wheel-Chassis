from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'handeye_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='yangxuan',
    maintainer_email='3465219188@qq.com',
    description='Hand-eye bridge: pixel detection → hand-eye transform → robot arm control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge_node = handeye_bridge.bridge_node:main',
        ],
    },
)
