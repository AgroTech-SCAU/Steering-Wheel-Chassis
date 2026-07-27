import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vison_topic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), ['README.md']),
        (os.path.join('share', package_name, 'resource'), ['resource/best.onnx']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'onnxruntime'],
    zip_safe=True,
    maintainer='yuuy',
    maintainer_email='yuuy@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_detect_server = vison_topic.detect_and_send:main',
            'vision_detect_client = vison_topic.vision_receiver:main',
        ],
    },
)
