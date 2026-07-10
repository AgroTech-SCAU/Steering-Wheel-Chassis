from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'atlas_autonomous_transport_manager'


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
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='yangxuan',
    maintainer_email='3465219188@qq.com',
    description='Atlas 智械争锋全自主运输区任务状态机与安全门控',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'autonomous_transport_manager = atlas_autonomous_transport_manager.autonomous_transport_manager:main',
            'voice_player = atlas_autonomous_transport_manager.voice_player:main',
        ],
    },
)
