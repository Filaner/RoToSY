from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'rotosy_gripper_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoToSY',
    maintainer_email='cjfgml0221@naver.com',
    description='Flange I/O gripper control examples for RoToSY Doosan E0509.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_electromagnet_gripper = rotosy_gripper_control.keyboard_electromagnet_gripper:main',
        ],
    },
)
