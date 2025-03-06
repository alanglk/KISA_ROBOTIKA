from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'mcl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
                      glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
                      glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'), 
                      glob(os.path.join('rviz', '*rviz'))),
        (os.path.join('share', package_name, 'maps'),
                      glob(os.path.join('maps', '*.*'))),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ccplaore',
    maintainer_email='ccplaore@gmail.com',
    description='TODO: Package description',
    license='APACHE-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mcl_node = mcl.mcl_node:main',
            'amcl_node = mcl.amcl_node:main',
            'odom2map_node = mcl.odom2maptf:main',
            'getmap = mcl.getmap:main',
            'laser2odom = mcl.transform_laser_points:main',
        ],
    },
)
