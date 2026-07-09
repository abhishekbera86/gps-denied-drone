import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'common_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'openvins'),
            glob('config/openvins/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhishek Bera',
    maintainer_email='a.bera.gcect@gmail.com',
    description='Localization-source selection and VIO odometry bridge feeding PX4 EKF2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'set_localization_source = common_perception.set_localization_source:main',
            'loopback_odometry_bridge = common_perception.loopback_odometry_bridge:main',
            'openvins_odometry_bridge = common_perception.openvins_odometry_bridge:main',
        ],
    },
)
