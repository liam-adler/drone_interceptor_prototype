import os
from glob import glob

from setuptools import find_packages, setup

package_name = "drone_interceptor"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "PyYAML",
    ],
    zip_safe=True,
    maintainer="liam",
    maintainer_email="liam.dlr@icloud.com",
    description=(
        "ROS 2 simulation package for target-interceptor pursuit experiments "
        "with baseline and MPC controllers."
    ),
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "target_behavior_node = drone_interceptor.nodes.target_behavior_node:main",
            "target_dynamics_node = drone_interceptor.nodes.target_dynamics_node:main",
            "estimator_node = drone_interceptor.nodes.estimator_node:main",
            "interceptor_dynamics_node = "
            "drone_interceptor.nodes.interceptor_dynamics_node:main",
            "interceptor_guidance_node = "
            "drone_interceptor.nodes.interceptor_guidance_node:main",
            "interceptor_mpc_node = drone_interceptor.nodes.interceptor_mpc_node:main",
            "interceptor_mpc_dynamics_node = "
            "drone_interceptor.nodes.interceptor_mpc_dynamics_node:main",
            "distance_monitor_node = drone_interceptor.nodes.distance_monitor_node:main",
            "results_logger_node = drone_interceptor.nodes.results_logger_node:main",
            "compare_results = scripts.compare_results:main",
            "organize_results = scripts.organize_results:main",
            "plot_results = scripts.plot_results:main",
            "run_experiment = scripts.run_experiment:main",
        ],
    },
)
