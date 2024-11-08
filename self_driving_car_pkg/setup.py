from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'self_driving_car_pkg'
config_mod = 'self_driving_car_pkg/config'
det_mod = 'self_driving_car_pkg/Detection'
l_det_mod = 'self_driving_car_pkg/Detection/Lanes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*')),
        (os.path.join('share', package_name,'worlds'), glob('worlds/*'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='piyush',
    maintainer_email='piyush@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_node = self_driving_car_pkg.drive_node:main',
            'video_save = self_driving_car_pkg.video_save:main',
            'spawner_node = self_driving_car_pkg.sdf_spawner:main',
            'computer_vision_node = self_driving_car_pkg.computer_vision_node:main',
        ],
    },
)
