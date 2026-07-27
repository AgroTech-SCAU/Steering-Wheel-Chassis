import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'atlas_autonomous_task'

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
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='yangxuan',
    maintainer_email='3465219188@qq.com',
    description='Atlas PI 端智械争锋全自主运输状态机',
    license='MIT',
    entry_points={
        'console_scripts': [
            'autonomous_task_node = atlas_autonomous_task.autonomous_task_node:main',
        ],
    },
)
