import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('playground')
    profile_path = LaunchConfiguration('bringup_profile').perform(context)

    if not os.path.isabs(profile_path) and not os.path.exists(profile_path):
        candidate = os.path.join(pkg_share, 'config', 'bringup', profile_path)
        if not candidate.endswith('.yaml'):
            candidate += '.yaml'
        if os.path.exists(candidate):
            profile_path = candidate

    profile_config = {}
    if os.path.exists(profile_path):
        with open(profile_path, 'r') as f:
            data = yaml.safe_load(f)
            if data and 'bringup' in data and 'ros__parameters' in data['bringup']:
                profile_config = data['bringup']['ros__parameters']
            elif data and isinstance(data, dict):
                profile_config = data
    else:
        import warnings
        warnings.warn(f"[bringup] Profile not found: '{profile_path}' — using defaults/CLI args")

    def get_param(name, default=''):
        val = context.launch_configurations.get(name, 'default')
        if val != 'default' and val != '':
            return val
        if name in profile_config:
            return profile_config[name]
        return default

    def get_flag(name, default='true'):
        return str(get_param(name, default)).lower()

    def get_config_path(name, default_rel_path):
        raw_path = str(get_param(name, default_rel_path))
        if os.path.isabs(raw_path):
            return raw_path
        return os.path.join(pkg_share, raw_path)

    use_sim_time          = get_flag('use_sim_time', 'false')
    is_sim                = (use_sim_time == 'true')

    launch_description    = 'false' if is_sim else get_flag('launch_description', 'true')
    launch_camera_00      = 'false' if is_sim else get_flag('launch_camera_00', 'true')
    launch_lidar_00       = 'false' if is_sim else get_flag('launch_lidar_00', 'true')
    launch_camera_00_odom = get_flag('launch_camera_00_odom', 'true')
    launch_lidar_00_odom  = get_flag('launch_lidar_00_odom', 'true')
    use_ukf               = get_flag('use_ukf', 'false')
    launch_localization   = get_flag('launch_localization', 'true')
    launch_mapping        = get_flag('launch_mapping', 'false')
    delete_db_on_start    = get_flag('delete_db_on_start', 'false')

    # Camera decode settings
    camera_00_enable_decode     = get_flag ('camera_00_enable_decode',     'false')
    camera_00_decode_backend    = get_param('camera_00_decode_backend',    'cpu')
    camera_00_decode_threads    = get_param('camera_00_decode_threads',    '4')
    camera_00_decode_queue_size = get_param('camera_00_decode_queue_size', '5')
    camera_00_decode_gpu_vendor = get_param('camera_00_decode_gpu_vendor', 'auto')

    # Config file resolving
    camera_00_config      = get_config_path('camera_00_config', 'config/sensors/oak_d_pro.yaml')
    lidar_00_config       = get_config_path('lidar_00_config', 'config/sensors/ouster_os.yaml')
    camera_00_odom_config = get_config_path('camera_00_odom_config', 'config/odometry/rgbd_odom.yaml')
    lidar_00_odom_config  = get_config_path('lidar_00_odom_config', 'config/odometry/icp_odom.yaml')

    default_ekf_rel       = 'config/localization/ekf_filter_sim.yaml' if is_sim else 'config/localization/ekf_filter.yaml'
    ekf_config            = get_config_path('ekf_config', default_ekf_rel)
    ukf_config            = get_config_path('ukf_config', 'config/localization/ukf_filter.yaml')
    mapping_config        = get_config_path('mapping_config', 'config/mapping/rtabmap_slam.yaml')

    # Includes
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'includes', 'description.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(launch_description)
    )

    camera_00_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'includes', 'luxonis_camera.launch.py')),
        launch_arguments={
            'camera_ns':          'camera_00',
            'camera_name':        'camera_00',
            'params_file':        camera_00_config,
            'enable_composable':  'true',
            'enable_republish':   'true',
            'enable_decode':      str(camera_00_enable_decode),
            'decode_backend':     str(camera_00_decode_backend),
            'decode_threads':     str(camera_00_decode_threads),     # <-- Fixed (forces string)
            'decode_queue_size':  str(camera_00_decode_queue_size), # <-- Fixed (forces string)
            'decode_gpu_vendor':  str(camera_00_decode_gpu_vendor),
        }.items(),
        condition=IfCondition(launch_camera_00)
    )

    lidar_00_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'includes', 'ouster_lidar.launch.py')),
        launch_arguments={
            'params_file': lidar_00_config,
            'ouster_ns': 'lidar_00'
        }.items(),
        condition=IfCondition(launch_lidar_00)
    )

    camera_00_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'includes', 'rgbd_odometry.launch.py')),
        launch_arguments={
            'camera_ns': 'camera_00',
            'params_file': camera_00_odom_config,
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(launch_camera_00_odom)
    )

    lidar_00_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'includes', 'icp_odometry.launch.py')),
        launch_arguments={
            'lidar_ns': 'lidar_00',
            'params_file': lidar_00_odom_config,
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(launch_lidar_00_odom)
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'includes', 'localization.launch.py')),
        launch_arguments={
            'use_ukf': use_ukf,
            'use_sim_time': use_sim_time,
            'ekf_config_file': ekf_config,
            'ukf_config_file': ukf_config
        }.items(),
        condition=IfCondition(launch_localization)
    )

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'includes', 'mapping.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': mapping_config,
            'delete_db_on_start': delete_db_on_start
        }.items(),
        condition=IfCondition(launch_mapping)
    )

    return [
        description_launch,
        camera_00_launch,
        lidar_00_launch,
        camera_00_odom_launch,
        lidar_00_odom_launch,
        localization_launch,
        mapping_launch,
    ]

def generate_launch_description():
    pkg_share = FindPackageShare('playground')
    default_profile = PathJoinSubstitution([pkg_share, 'config', 'bringup', 'bringup_profile.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'bringup_profile',
            default_value=default_profile,
            description='Path to YAML profile configuring subsystem toggles and configs'
        ),
        DeclareLaunchArgument('use_sim_time', default_value='default'),
        DeclareLaunchArgument('launch_description', default_value='default'),
        DeclareLaunchArgument('launch_camera_00', default_value='default'),
        DeclareLaunchArgument('launch_lidar_00', default_value='default'),
        DeclareLaunchArgument('launch_camera_00_odom', default_value='default'),
        DeclareLaunchArgument('launch_lidar_00_odom', default_value='default'),
        DeclareLaunchArgument('launch_localization', default_value='default'),
        DeclareLaunchArgument('use_ukf', default_value='default'),
        DeclareLaunchArgument('launch_mapping', default_value='default'),
        DeclareLaunchArgument('camera_00_config', default_value='default'),
        DeclareLaunchArgument('lidar_00_config', default_value='default'),
        DeclareLaunchArgument('camera_00_odom_config', default_value='default'),
        DeclareLaunchArgument('lidar_00_odom_config', default_value='default'),
        DeclareLaunchArgument('ekf_config', default_value='default'),
        DeclareLaunchArgument('ukf_config', default_value='default'),
        DeclareLaunchArgument('mapping_config', default_value='default'),
        DeclareLaunchArgument('delete_db_on_start', default_value='default'),

        DeclareLaunchArgument('camera_00_enable_decode',     default_value='default'),
        DeclareLaunchArgument('camera_00_decode_backend',    default_value='default'),
        DeclareLaunchArgument('camera_00_decode_threads',    default_value='default'),
        DeclareLaunchArgument('camera_00_decode_queue_size', default_value='default'),
        DeclareLaunchArgument('camera_00_decode_gpu_vendor', default_value='default'),

        OpaqueFunction(function=launch_setup)
    ])

