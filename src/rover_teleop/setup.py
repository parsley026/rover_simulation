import os
import glob
from setuptools import find_packages, setup

package_name = 'rover_teleop'

def get_files(path):
    """
    Collect all file paths under a directory recursively.
    
    Parameters:
    	path (str): Root directory to search.
    
    Returns:
    	list: File paths found under the specified directory.
    """
    return [f for f in glob.glob(f'{path}/**/*', recursive=True) if os.path.isfile(f)]

config_files = get_files('config')
launch_files = get_files('launch')
run_files    = get_files('run')
urdf_files   = get_files('urdf')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', config_files),
        ('share/' + package_name + '/launch', launch_files),
        ('share/' + package_name + '/run', run_files),
        ('share/' + package_name + '/urdf', urdf_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raptors',
    maintainer_email='raptors@todo.todo',
    description='Teleop and macro nodes for the rover',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_keyboard = rover_teleop.teleop_keyboard:main',
            'teleop_macro = rover_teleop.teleop_macro:main',        # <--- ADDED MACRO SCRIPT HERE!
        ],
    },
)