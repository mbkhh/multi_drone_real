import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'swarm_single_decenter'

setup(
	name=package_name,
	version='1.0.0',
	packages=find_packages(exclude=['test']),
	data_files=[
		('share/ament_index/resource_index/packages',
			['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
		('share/' + package_name, ['README.md']),
		(os.path.join('share', package_name, 'launch'),
			glob(os.path.join('launch', '*launch.py'))),
	],
	install_requires=['setuptools'],
	extras_require={'test': ['pytest']},
	zip_safe=True,
	maintainer='mahdi-roohi',
	maintainer_email='99536958+mrwhy224@users.noreply.github.com',
	description='Decentralized multi-drone controller with independent missions.',
	license='Apache License 2.0',
	entry_points={
		'console_scripts': [
			'control_node = swarm_single_decenter.single_control_node:main',
			'testForm = swarm_single_decenter.swarm_formation:main',
			'test = swarm_single_decenter.local_state_publisher:main',
			'test2 = swarm_single_decenter.local_state_monitor:main',
		],
	},
)
