import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hospital_sim'

# Definimos los archivos básicos
data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    
    # 1. Copiar archivos de mundo (.sdf / .world)
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    
    # 2. Copiar archivos de launch (para cuando crees el launcher)
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
]

# 3. FUNCIÓN MÁGICA: Copiar recursivamente la carpeta 'models'
# Esto es necesario porque 'data_files' no copia carpetas enteras por defecto.
def package_files(data_files, directory_name):
    paths = []
    for (path, directories, filenames) in os.walk(directory_name):
        for filename in filenames:
            file_path = os.path.join(path, filename)
            install_path = os.path.join('share', package_name, path)
            data_files.append((install_path, [file_path]))
    return data_files

# Ejecutamos la función para añadir todos los modelos a la instalación
if os.path.exists('models'):
    data_files = package_files(data_files, 'models')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files, # Aquí pasamos la lista completa
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
