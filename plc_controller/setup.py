from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'plc_controller'

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
    install_requires=['setuptools', 'pymodbus'],
    zip_safe=True,
    maintainer='RoToSY',
    maintainer_email='cjfgml0221@naver.com',
    description='LS ELECTRIC XBC-DR10E PLC bridge (Modbus TCP)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plc_bridge = plc_controller.plc_bridge_node:main',
        ],
    },
)
