from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'mobile_simulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'materials', 'meshes'), glob('materials/meshes/*')),
        (os.path.join('share', package_name, 'materials', 'scripts'), glob('materials/scripts/*')),
        (os.path.join('share', package_name, 'materials', 'textures'), glob('materials/textures/*')),
        (os.path.join('share', package_name, 'models', 'mobile_delivery_robot'), glob('models/mobile_delivery_robot/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoToSY',
    maintainer_email='cjfgml0221@naver.com',
    description='Mobile robot Gazebo and Nav2 simulation for RoToSY.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo_goal_sender = mobile_simulation.demo_goal_sender:main',
        ],
    },
)
