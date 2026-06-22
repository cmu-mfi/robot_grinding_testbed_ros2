from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rgt_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.glb')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexander Minor',
    maintainer_email='alexander@minor.pw',
    description='RGT Manager',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'rgt_manager = rgt_manager.rgt_manager:main',
        ],
    },
)
