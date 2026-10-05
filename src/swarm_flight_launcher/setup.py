from setuptools import find_packages, setup


package_name = 'swarm_flight_launcher'

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
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='mahdi-roohi',
    maintainer_email='99536958+mrwhy224@users.noreply.github.com',
    description=(
        'Desktop SSH launcher and live output viewer for the real swarm stack.'
    ),
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'launcher = swarm_flight_launcher.gui:main',
        ],
    },
)
