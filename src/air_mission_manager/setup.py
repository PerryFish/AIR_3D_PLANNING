from setuptools import setup

package_name = "air_mission_manager"

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
    description="Aerial mission manager.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mission_manager = air_mission_manager.mission_manager:main",
        ],
    },
)
