from setuptools import find_packages, setup

package_name = "kr10_taller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SMS",
    maintainer_email="santiago.machado@eia.edu.co",
    description="Scripts del Taller ROS2/MoveIt2 con el KUKA KR10 R1100-2",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ik_poses = kr10_taller.ik_poses:main",
        ],
    },
)
