from setuptools import find_packages, setup

package_name = 'swarm_single'

setup(
	name=package_name,
	version='1.0.0',
	packages=find_packages(exclude=['test']),
	data_files=[
		('share/ament_index/resource_index/packages',
			['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
	],
	install_requires=['setuptools'],
	zip_safe=True,
	maintainer='mahdi-roohi',
	maintainer_email='99536958+mrwhy224@users.noreply.github.com',
	description='A package to launch and control a single drone instance for swarm simulations.',
	license='Apache License 2.0',
	entry_points={
		'console_scripts': [
			'control_node = swarm_single.single_control_node:main',
			'testForm = swarm_single.swarm_formation:main',
			'test = swarm_single.test:main',
			'test2 = swarm_single.test2:main',
		],
	},
)
