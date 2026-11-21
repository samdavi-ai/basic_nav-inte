import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gracemo_rl'
canonical_places = os.path.join('..', '..', '..', 'config', 'places.json')

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [str(canonical_places)]),
        (os.path.join('share', package_name, 'launch'), glob('gracemo_rl/launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('gracemo_rl/config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('gracemo_rl/models/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sabarishchandrasekaran',
    maintainer_email='sabarishchandrasekaran@todo.todo',
    description='Reinforcement Learning Local Planner for GRaCEmo ViRa Autonomous Robot',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_local_planner = gracemo_rl.nodes.rl_local_planner:main',
        ],
    },
)

