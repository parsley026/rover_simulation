import glob

from setuptools import find_packages, setup

package_name = 'rosa'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob.glob('config/*')),
        ('share/' + package_name + '/launch', glob.glob('launch/*')),
    ],
    install_requires=[
        'setuptools',
        'langchain',
        'langchain-core',
        'langchain-community',
        'langchain-openai',
    ],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@todo.todo',
    description=(
        'ROSA (Robot Operating System Agent) — AI agent framework '
        'for natural language interaction with ROS systems.'
    ),
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
        'openai': ['langchain-openai'],
        'anthropic': ['langchain-anthropic'],
        'ollama': ['langchain-ollama'],
    },
    entry_points={
        'console_scripts': [
            'rosa_agent = rosa.agent_node:main',
            'rosa_system_monitor = rosa.system_monitor_node:main',
            'rosa_diagnostics_bringup = rosa.diagnostics.bringup_diagnostics:main',
            'rosa_diag_ros2 = rosa.diagnostics.diag_ros2:main',
            'rosa_diag_calc = rosa.diagnostics.diag_calculation:main',
            'rosa_diag_log = rosa.diagnostics.diag_log:main',
            'rosa_diag_sys = rosa.diagnostics.diag_system:main',
            'web_action_proxy = rosa.web_action_proxy:main',
        ],
    },
)

