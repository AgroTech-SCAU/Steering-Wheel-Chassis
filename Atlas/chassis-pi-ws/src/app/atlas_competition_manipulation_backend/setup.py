from setuptools import find_packages, setup

package_name = 'atlas_competition_manipulation_backend'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/manipulation.yaml']),
        ('share/' + package_name + '/launch', ['launch/manipulation_backend.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AgroTech-SCAU',
    maintainer_email='1219921425@qq.com',
    description='Competition manipulation orchestration backend for Atlas.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'competition_manipulation_backend = atlas_competition_manipulation_backend.backend:main',
        ],
    },
)
