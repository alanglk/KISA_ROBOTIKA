from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'ekf_localization'

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
                      glob(os.path.join('config', '*.*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ccplaore',
    maintainer_email='ccplaore@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_loc_unknown = ekf_localization.EKFlocUnknownCorrespondences:main',
            # 'ekf_loc_known = ekf_localization.EKFlocKnownCorrespondences:main',
            'ekfplot = ekf_localization.ekfplot:main',
            'odomtest = ekf_localization.odom_test:main'
        ],
    },
)
