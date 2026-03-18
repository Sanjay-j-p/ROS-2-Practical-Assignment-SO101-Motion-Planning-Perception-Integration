
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'so101_state_machine'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
        ('share/' + package_name + '/config',
            glob('config/*.yaml') if os.path.exists('config') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanjay',
    maintainer_email='sanjayprabakar1002@gmail.com',
    description='SO101 pick-and-place: perception + BT + MoveIt',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bt_node          = so101_state_machine.bt_node:main',
            'perception_node  = so101_state_machine.perception_node:main',
        ],
    },
)