from setuptools import find_packages, setup

package_name = "perception_tower"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/tower_params.yaml"]),
        ("share/" + package_name + "/launch", ["launch/tower.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="perception_tower",
    maintainer_email="dev@example.com",
    description="Perception tower control and turntable scan/stitch.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tower_node = perception_tower.tower_node:main",
        ],
    },
)
