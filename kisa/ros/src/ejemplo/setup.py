from setuptools import find_packages, setup

package_name = 'ejemplo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='ag6154lk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lector = ejemplo.lector:main',
            'escritor = ejemplo.escritor:main',
            'lector_sphere = ejemplo.lector_sphere:main',
            'escritor_sphere = ejemplo.escritor_sphere:main'
        ],
    },
)
