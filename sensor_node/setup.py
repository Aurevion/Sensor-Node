from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'sensor_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    zip_safe=True,
    maintainer='solomon',
    maintainer_email='solomon@todo.todo',
    description='Sensor communication',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = sensor_node.sensor_node:main',
        ],
    },

)
