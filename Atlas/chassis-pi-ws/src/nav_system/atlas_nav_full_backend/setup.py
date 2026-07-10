from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'atlas_nav_full_backend'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yangxuan',
    maintainer_email='3465219188@qq.com',
    description='Atlas 任务状态机使用的 Nav2 完整导航后端适配器',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'full_nav_backend = atlas_nav_full_backend.full_nav_backend:main',
        ],
    },
)
