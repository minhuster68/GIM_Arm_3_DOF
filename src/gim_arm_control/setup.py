import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gim_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minh',
    maintainer_email='minhvuviet20051311123456789@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'motor_test_all_modes = gim_control.motor_test_all_modes:main',
            'origin_draw_trajectory = gim_control.origin_draw_trajectory:main',
            'sniff_can_ff = gim_control.sniff_can_ff:main',
            'check_gravity_model = gim_control.check_gravity_model:main',
        ],
    },
)