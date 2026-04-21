from glob import glob

import os

from setuptools import find_packages, setup

package_name = "igvc_hall_odom"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Elijah Spinner",
    maintainer_email="e-spinner@onu.edu",
    description="Hall-effect wheel odometry for IGVC",
    license="MIT",
    entry_points={
        "console_scripts": [
            "hall_odom_node = igvc_hall_odom.hall_odom_node:main",
        ],
    },
)
