import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'swarm_sim'

setup(
	name=package_name,
	version='1.0.0',
	packages=find_packages(exclude=['test']),
	data_files=[
		('share/ament_index/resource_index/packages',
			['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
		(os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
	],
	install_requires=['setuptools'],
	zip_safe=True,
	maintainer='mahdi-roohi',
	maintainer_email='99536958+mrwhy224@users.noreply.github.com',
	description='A ROS 2 package for simulating a drone swarm in Gazebo.',
	license='Apache License 2.0',
	tests_require=['pytest'],
	entry_points={
		'console_scripts': [
			'tf_publisher = swarm_sim.tf_publisher:main',
		],
	},
)
