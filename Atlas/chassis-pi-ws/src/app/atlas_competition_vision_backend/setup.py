from glob import glob
import os

from setuptools import find_packages, setup


package_name = "atlas_competition_vision_backend"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="AgroTech-SCAU",
    maintainer_email="1219921425@qq.com",
    description="Thin competition vision backend for Atlas mission services",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "vision_backend = atlas_competition_vision_backend.backend:main",
        ],
    },
)
