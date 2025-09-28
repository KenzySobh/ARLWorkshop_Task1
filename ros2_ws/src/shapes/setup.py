from setuptools import setup
import os
from glob import glob

package_name = 'shapes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kenzy', 
 maintainer_email='kenzysobh74503@gmail.com',
    description='Shapes drawing with turtlesim',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'shape_node = shapes.shape_node:main',
            'turtle_commander = shapes.turtle_commander:main',
        ],
    },
)