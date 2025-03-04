import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'occgrid'

# data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ]

# def package_files(data_files, directory_list):
#     paths_dict = {}

#     for directory in directory_list:
#         for (path, directories, filenames) in os.walk(directory):
#             for filename in filenames:

#                 file_path = os.path.join(path, filename)
#                 install_path = os.path.join('share', package_name, path)

#                 if install_path in paths_dict.keys():
#                     paths_dict[install_path].append(file_path)
#                 else:
#                     paths_dict[install_path] = [file_path]

#     for key in paths_dict.keys():
#         data_files.append((key, paths_dict[key]))

#     return data_files


setup(
    name=package_name,
    version='0.0.0',
    # packages to export
    packages=[package_name], 
    # files we want to install, specifically launch files
    
    data_files=[
        # Install marker file in the package index
        ('share/'+ package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', 
         ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch','*.launch.py'))),        
    ],
    # This is important aswell
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bee',
    maintainer_email='bee@todo.todo',
    description='Basic occupancy grid mapping algorithm in python',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'occgrid_mapping = occgrid.occgrid_mapping:main'
        ],
    },
)
