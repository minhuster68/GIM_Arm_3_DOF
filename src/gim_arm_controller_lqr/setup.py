import os
from glob import glob

from setuptools import find_packages, setup

package_name = "gim_arm_controller_lqr"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="minh",
    maintainer_email="minhvuviet20051311123456789@gmail.com",
    description="Time-varying LQR torque controller for GIM Arm 3-DOF",
    license="TODO",
    entry_points={
        "console_scripts": ["lqr_node = gim_arm_controller_lqr.node:main"],
    },
)
