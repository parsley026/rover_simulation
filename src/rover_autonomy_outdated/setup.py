import glob
from setuptools import find_packages, setup

package_name = 'rover_autonomy_outdated'

config_files = glob.glob('config/**/*', recursive=True)

launch_files = glob.glob('launch/**/*', recursive=True)
run_files    = glob.glob('run/**/*', recursive=True)

urdf_files   = glob.glob('urdf/**/*', recursive=True)


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
            'topography_global_node = rover_autonomy_outdated.topography_global_node:main',
            'topography_local_node = rover_autonomy_outdated.topography_local_node:main',
        ],
    },
)
