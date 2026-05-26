from setuptools import setup

package_name = "air_trajectory_generator"

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
    description="3D trajectory generator.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "trajectory_generator = air_trajectory_generator.trajectory_generator:main",
        ],
    },
)
