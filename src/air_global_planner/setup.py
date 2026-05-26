from setuptools import setup

package_name = "air_global_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nuaa",
    maintainer_email="nuaa@example.com",
    description="3D A* aerial global planner.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "astar_3d_planner = air_global_planner.astar_3d_planner:main",
        ],
    },
)
