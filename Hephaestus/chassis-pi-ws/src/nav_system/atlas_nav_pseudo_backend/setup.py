from setuptools import setup
from glob import glob

package_name = 'atlas_nav_pseudo_backend'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yangxuan',
    maintainer_email='3465219188@qq.com',
    description='Odom-based pseudo navigation backend for Atlas Pi mission stack.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pseudo_nav_backend = atlas_nav_pseudo_backend.pseudo_nav_backend:main',
        ],
    },
)
