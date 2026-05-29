from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'doosan_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoToSY',
    maintainer_email='cjfgml0221@naver.com',
    description='Doosan E0509 arm controller node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller  = doosan_controller.arm_controller_node:main',
            'test_movel      = doosan_controller.test_movel:main',
            'motion_sequence = doosan_controller.motion_sequence:main',
            'tcp_monitor     = doosan_controller.tcp_monitor:main',
        ],
    },
)
