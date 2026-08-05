from setuptools import find_packages, setup

package_name = 'swarm_station'

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
	description='A ROS 2 package to act as a ground control station, sending high-level commands to the drone swarm.',
	license='Apache License 2.0',
	entry_points={
		'console_scripts': [
			'test=swarm_station.test_node:main',
			'manual=swarm_station.manual_controller:main',
			'lidar=swarm_station.lidar_sensor:main',
			'station=swarm_station.station_node:main',
			'takeoff_spacing=swarm_station.takeoff_spacing_node:main',
		],
	},
)
