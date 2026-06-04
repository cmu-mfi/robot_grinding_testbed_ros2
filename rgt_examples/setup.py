from setuptools import find_packages, setup

package_name = 'rgt_examples'

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
    description='Examples for the rgt',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'tool_change_example = rgt_examples.tool_change_example:main',
            'move_till_force = rgt_examples.move_till_force:main'
        ],
    },
)
