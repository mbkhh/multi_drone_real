from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'swarm_p900_bridge'


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='mahdi-roohi',
    maintainer_email='99536958+mrwhy224@users.noreply.github.com',
    description=(
        'Selected ROS 2 topic bridge over a transparent P900 serial radio.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge_node = swarm_p900_bridge.bridge_node:main',
        ],
    },
)
