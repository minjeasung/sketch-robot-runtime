from glob import glob
import os

from setuptools import find_packages, setup


package_name = "rbpodo_painting_control"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Minjea",
    maintainer_email="minjea@example.com",
    description="Painting roller target publishers for ROS 2 admittance/contact control.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            (
                "painting_wrench_reference_node = "
                "rbpodo_painting_control.painting_wrench_reference_node:main"
            ),
            (
                "painting_force_monitor_node = "
                "rbpodo_painting_control.painting_force_monitor_node:main"
            ),
            (
                "painting_wrench_guard_node = "
                "rbpodo_painting_control.painting_wrench_guard_node:main"
            ),
            (
                "painting_segment_mode_node = "
                "rbpodo_painting_control.painting_segment_mode_node:main"
            ),
        ],
    },
)
