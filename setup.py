import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hospital_sim'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
]

def package_files(data_files, directory_name):
    for (path, directories, filenames) in os.walk(directory_name):
        for filename in filenames:
            file_path = os.path.join(path, filename)
            install_path = os.path.join('share', package_name, path)
            data_files.append((install_path, [file_path]))
    return data_files

for directory in ['models', 'meshes', 'urdf', 'worlds', 'config']:
    if os.path.exists(directory):
        data_files = package_files(data_files, directory)

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='Hospital Simulation Migration',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
