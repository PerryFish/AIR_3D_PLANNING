from glob import glob
from setuptools import setup

package_name = "aerial_exploration_planner"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nuaa",
    maintainer_email="nuaa@example.com",
    description="Minimal aerial 3D autonomous exploration validation package.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "synthetic_mapping_node = aerial_exploration_planner.synthetic_mapping_node:main",
            "aerial_exploration_node = aerial_exploration_planner.aerial_exploration_node:main",
            "exploration_metrics_node = aerial_exploration_planner.exploration_metrics_node:main",
            "mode_manager_node = aerial_exploration_planner.mode_manager_node:main",
        ],
    },
)
