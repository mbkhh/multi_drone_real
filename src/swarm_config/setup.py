import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'swarm_config'

setup(
	name=package_name,
	version='1.0.0',
	packages=find_packages(exclude=['test']),
	data_files=[
		('share/ament_index/resource_index/packages',
			['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
		(os.path.join('share', package_name, 'config'), glob('config/*.yaml.dist')),
		(os.path.join('share', package_name, 'config/scenarios'), glob('config/scenarios/*.yaml')),
	],
	install_requires=['setuptools', 'pyyaml'],
	zip_safe=True,
	maintainer='mahdi-roohi',
	maintainer_email='99536958+mrwhy224@users.noreply.github.com',
	description='A ROS 2 package to manage all configuration files and parameters for the drone swarm project.',
	license='Apache License 2.0',
	tests_require=['pytest'],
	entry_points={
		'console_scripts': [
			'config = swarm_config.test:main',
		],
	},
)
