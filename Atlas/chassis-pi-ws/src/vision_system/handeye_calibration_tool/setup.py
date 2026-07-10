from glob import glob
from setuptools import find_packages, setup

package_name = "handeye_calibration_tool"

setup(
    name=package_name,
    version="0.1.2",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kaede Rei",
    maintainer_email="kaerei@foxmail.com",
    description="UVC 相机与机械臂交互式手眼标定工具",
    license="MIT",
    entry_points={
        "console_scripts": [
            "handeye_tool = handeye_calibration_tool.handeye_tool:main",
        ],
    },
)
