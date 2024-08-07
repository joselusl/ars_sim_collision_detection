from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ars_sim_collision_detection'

setup(
 name=package_name,
 version='0.0.1',
 packages=find_packages(exclude=['test']),
 data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        #(os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
	],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Jose Luis SANCHEZ-LOPEZ',
 maintainer_email='joseluis.sanlop@gmail.com',
 description='TODO: Package description',
 license='BSD',
 tests_require=['pytest'],
 entry_points={'console_scripts': [
 		'ars_sim_collision_detection_ros_node = ars_sim_collision_detection.ars_sim_collision_detection_ros_node:main',
        ],},
)
