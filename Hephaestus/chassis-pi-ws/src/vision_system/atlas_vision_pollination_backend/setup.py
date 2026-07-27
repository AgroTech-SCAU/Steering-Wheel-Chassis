from setuptools import setup
from glob import glob

package_name = 'atlas_vision_pollination_backend'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'PyYAML', 'numpy'],
    zip_safe=True,
    maintainer='yangxuan',
    maintainer_email='3465219188@qq.com',
    description='Atlas 树莓派视觉授粉后端',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vision_pollination_backend = atlas_vision_pollination_backend.vision_pollination_backend:main',
            'camera_target_service = atlas_vision_pollination_backend.camera_target_service:main',
        ],
    },
)
