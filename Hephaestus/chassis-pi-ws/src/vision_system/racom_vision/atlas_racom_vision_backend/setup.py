from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'atlas_racom_vision_backend'

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
    description='RACOM/Raicom 视觉检测结果到 Atlas 任务视觉目标接口的适配器',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'racom_camera_target_service = atlas_racom_vision_backend.racom_camera_target_service:main',
        ],
    },
)
