import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'navigation_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cmtabr',
    maintainer_email='caio.abreu@sou.inteli.edu.br',
    description='This package does navigation controlling of turtlebot3',
    license='CC0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'set_origin = navigation_package.set_origin:main',
	        'navigation_controller = navigation_package.navigation_controller:main'
        ],
    },
)
