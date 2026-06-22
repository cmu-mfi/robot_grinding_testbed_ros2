from setuptools import find_packages, setup

package_name = 'rgt_scripts'

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
    maintainer='Alexander Minor',
    maintainer_email='alexander@minor.pw',
    description='Scripts for the rgt',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'find_tray_reference = rgt_scripts.find_tray_reference:main',
            'test_tray_reference = rgt_scripts.test_tray_reference:main',
            'find_world_reference = rgt_scripts.find_world_reference:main',
            'spacemouse = rgt_scripts.spacemouse:main'
        ],
    },
)
