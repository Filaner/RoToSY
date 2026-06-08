from setuptools import find_packages, setup

package_name = 'web_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/web_interface']),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={
        'web_interface': ['static/*'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cheol',
    maintainer_email='cjfgml0221@naver.com',
    description='Web-based robot arm control interface for RoToSY',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'web_server = web_interface.main:main',
        ],
    },
)
