from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'rotosy_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoToSY',
    maintainer_email='cjfgml0221@naver.com',
    description='Camera-to-robot TF calibration utilities for RoToSY.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_camera_calibrator   = rotosy_calibration.aruco_camera_calibrator:main',
            'calibration_motion_runner = rotosy_calibration.calibration_motion_runner:main',
            'touch_calibration         = rotosy_calibration.touch_calibration:main',
        ],
    },
)
