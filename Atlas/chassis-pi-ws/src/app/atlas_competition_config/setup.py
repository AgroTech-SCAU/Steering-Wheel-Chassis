from setuptools import find_packages, setup

package_name = "atlas_competition_config"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AgroTech-SCAU",
    maintainer_email="1219921425@qq.com",
    description="Shared top-level competition YAML parser for Atlas backends.",
    license="Apache-2.0",
)
