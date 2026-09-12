import os

from setuptools import find_packages, setup

package_name = 'swarm_vision'
model_source = os.path.join('..', '..', 'yolo11n.onnx')
model_data = []
if os.path.isfile(os.path.join(os.path.dirname(__file__), model_source)):
    model_data.append(
        (f'share/{package_name}/models', [model_source])
    )

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + model_data,
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='ROS 2 YOLO ONNX vision nodes for the drone swarm.',
    license='Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_node = swarm_vision.vision_node:main',
            'flight_vision_node = swarm_vision.flight_vision_node:main',
        ],
    },
)
