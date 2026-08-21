import os
import glob
from setuptools import find_packages, setup

package_name = 'playground'

config_sensors = glob.glob('config/sensors/*.*')
config_odometry = glob.glob('config/odometry/*.*')
config_localization = glob.glob('config/localization/*.*')
config_mapping = glob.glob('config/mapping/*.*')
config_bringup = glob.glob('config/bringup/*.*')

launch_root = glob.glob('launch/*.*')
launch_includes = glob.glob('launch/includes/*.*')
launch_archive = glob.glob('launch/archive/*.*')

urdf_files = glob.glob('urdf/*.*')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config/sensors', config_sensors),
        ('share/' + package_name + '/config/odometry', config_odometry),
        ('share/' + package_name + '/config/localization', config_localization),
        ('share/' + package_name + '/config/mapping', config_mapping),
        ('share/' + package_name + '/config/bringup', config_bringup),
        ('share/' + package_name + '/launch', launch_root),
        ('share/' + package_name + '/launch/includes', launch_includes),
        ('share/' + package_name + '/launch/archive', launch_archive),
        ('share/' + package_name + '/urdf', urdf_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rex',
    maintainer_email='rex@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cpu_image_decoder  = playground.components.cpu_image_decoder:main',
            'gpu_nvidia_decoder = playground.components.gpu_nvidia_decoder:main',
            'gpu_amd_decoder    = playground.components.gpu_amd_decoder:main',
            'aruco_localization_node = playground.components.aruco_localization_node:main',
        ],
    },
)
