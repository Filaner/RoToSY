from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dsr_safety_recovery'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('lib/python3.10/site-packages', package_name, 'static'),
            glob('dsr_safety_recovery/static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cheol',
    maintainer_email='cjfgml0221@naver.com',
    description='DSR E0509 safety recovery node with web UI',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'safety_recovery = dsr_safety_recovery.node:main',
        ],
    },
)
