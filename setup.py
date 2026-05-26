import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drone_interceptor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liam',
    maintainer_email='liam.dlr@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    "console_scripts": [
        "target_behavior_node = drone_interceptor.target_behavior_node:main",
        "target_dynamics_node = drone_interceptor.target_dynamics_node:main",
        "interceptor_dynamics_node = drone_interceptor.interceptor_dynamics_node:main",
        "interceptor_guidance_node = drone_interceptor.interceptor_guidance_node:main",
        "distance_monitor_node = drone_interceptor.distance_monitor_node:main",
    ],
},
)
