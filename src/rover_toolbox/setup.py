import os
from glob import glob
from setuptools import setup

package_name = 'rover_toolbox'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # CRITICAL: This line copies your yaml file into the ROS 2 install directory
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@domain.com',
    description='Toolbox for rover simulation analysis',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # CRITICAL: This registers your executable
            'odom_analyzer = rover_toolbox.odom_analyzer:main', 
        ],
    },
)