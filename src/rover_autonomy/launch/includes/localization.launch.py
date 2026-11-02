import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'),
        value_type=bool
    )
    
    use_ekf = LaunchConfiguration('use_ekf').perform(context).lower() == 'true'
    use_ukf = LaunchConfiguration('use_ukf').perform(context).lower() == 'true'
    ekf_config_path = LaunchConfiguration('ekf_params_file').perform(context)
    ukf_config_path = LaunchConfiguration('ukf_params_file').perform(context)

    localization_ns = LaunchConfiguration('localization_ns').perform(context)
    localization_mode = str(LaunchConfiguration('localization_mode').perform(context))
    lidar_ns = LaunchConfiguration('lidar_ns').perform(context)
    cam0_ns = LaunchConfiguration('camera_00_ns').perform(context)
    cam1_ns = LaunchConfiguration('camera_01_ns').perform(context)

    launch_aruco_00 = LaunchConfiguration('launch_aruco_00').perform(context).lower() == 'true'
    launch_aruco_01 = LaunchConfiguration('launch_aruco_01').perform(context).lower() == 'true'
    launch_aruco_02 = LaunchConfiguration('launch_aruco_02').perform(context).lower() == 'true'

    # Config for continuous sensors (Velocity only)
    odom_velocity_config = [False, False, False,
                            False, False, False,
                            True,  True,  False,  # vx, vy
                            False, False, True,   # vyaw
                            False, False, False]

    # Config for IMU (Attitude and Angular Velocity)
    imu_config = [False, False, False,
                  True,  True,  False,  # Roll, Pitch
                  False, False, False,
                  True,  True,  True,   # Angular velocities
                  True,  False, False]  # Linear accel X (optional)

    mode_params = {}

    if localization_mode == '111':
        # 111: Full Fusion (LiDAR Vel + Cam0 Vel + Cam1 Vel)
        mode_params = {
            'odom0': f"/{lidar_ns}/odom",  'odom0_config': odom_velocity_config,  'odom0_relative': False,
            'odom1': f"/{cam0_ns}/odom",   'odom1_config': odom_velocity_config,  'odom1_relative': False,
            'odom2': f"/{cam1_ns}/odom",   'odom2_config': odom_velocity_config,  'odom2_relative': False,
            'imu0': f"/{lidar_ns}/imu",    'imu0_config': imu_config, 'imu0_queue_size': 50, 'imu0_relative': False, 'imu0_remove_gravitational_acceleration': True,
        }
    elif localization_mode == '110':
        # 110: LiDAR Vel + Cam0 Vel
        mode_params = {
            'odom0': f"/{lidar_ns}/odom",  'odom0_config': odom_velocity_config,  'odom0_relative': False,
            'odom1': f"/{cam0_ns}/odom",   'odom1_config': odom_velocity_config,  'odom1_relative': False,
            'imu0': f"/{lidar_ns}/imu",    'imu0_config': imu_config, 'imu0_queue_size': 50, 'imu0_relative': False, 'imu0_remove_gravitational_acceleration': True,
        }
    elif localization_mode == '101':
        # 101: LiDAR Vel + Cam1 Vel
        mode_params = {
            'odom0': f"/{lidar_ns}/odom",  'odom0_config': odom_velocity_config,  'odom0_relative': False,
            'odom1': f"/{cam1_ns}/odom",   'odom1_config': odom_velocity_config,  'odom1_relative': False,
            'imu0': f"/{lidar_ns}/imu",    'imu0_config': imu_config, 'imu0_queue_size': 50, 'imu0_relative': False, 'imu0_remove_gravitational_acceleration': True,
        }
    elif localization_mode == '100':
        # 100: LiDAR Vel Only
        mode_params = {
            'odom0': f"/{lidar_ns}/odom",  'odom0_config': odom_velocity_config,  'odom0_relative': False,
            'imu0': f"/{lidar_ns}/imu",    'imu0_config': imu_config, 'imu0_queue_size': 50, 'imu0_relative': False, 'imu0_remove_gravitational_acceleration': True,
        }
    elif localization_mode in ['011', '11', '9']:
        # 011: Dual Camera Vel
        mode_params = {
            'odom0': f"/{cam0_ns}/odom",   'odom0_config': odom_velocity_config,  'odom0_relative': False,
            'odom1': f"/{cam1_ns}/odom",   'odom1_config': odom_velocity_config,  'odom1_relative': False,
        }
    elif localization_mode in ['010', '10', '8']:
        # 010: Cam0 Vel Only
        mode_params = {
            'odom0': f"/{cam0_ns}/odom",   'odom0_config': odom_velocity_config,  'odom0_relative': False,
        }
    elif localization_mode in ['001', '1']:
        # 001: Cam1 Vel Only
        mode_params = {
            'odom0': f"/{cam1_ns}/odom",   'odom0_config': odom_velocity_config,  'odom0_relative': False,
        }

    # Config for ArUco absolute pose reference (Pose: x, y, yaw)
    aruco_pose_config = [True,  True,  False,
                         False, False, True,
                         False, False, False,
                         False, False, False,
                         False, False, False]

    if launch_aruco_00:
        mode_params['pose0'] = '/aruco_00/pose'
        mode_params['pose0_config'] = aruco_pose_config
        mode_params['pose0_differential'] = False
    
    if launch_aruco_01:
        mode_params['pose1'] = '/aruco_01/pose'
        mode_params['pose1_config'] = aruco_pose_config
        mode_params['pose1_differential'] = False

    if launch_aruco_02:
        mode_params['pose2'] = '/aruco_02/pose'
        mode_params['pose2_config'] = aruco_pose_config
        mode_params['pose2_differential'] = False

    # Add standard queue sizes dynamically
    for key in list(mode_params.keys()):
        if 'config' in key:
            base_name = key.replace('_config', '')
            if f"{base_name}_queue_size" not in mode_params:
                mode_params[f"{base_name}_queue_size"] = 10
            if f"{base_name}_differential" not in mode_params and 'pose' not in base_name:
                mode_params[f"{base_name}_differential"] = False

    nodes = []

    if use_ekf:
        nodes.append(Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=localization_ns,
            output='screen',
            parameters=[
                ekf_config_path, 
                {'use_sim_time': use_sim_time}, 
                mode_params
            ],
            arguments=['--ros-args', '--log-level', 'warn'],
        ))

    if use_ukf:
        nodes.append(Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node',
            namespace=localization_ns,
            output='screen',
            parameters=[
                ukf_config_path, 
                {'use_sim_time': use_sim_time}, 
                mode_params
            ],
            arguments=['--ros-args', '--log-level', 'warn'],
            remappings=[('odometry/filtered', 'odometry/filtered_ukf')]
        ))

    return nodes

def generate_launch_description():
    pkg_share = FindPackageShare('rover_autonomy')
    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'odometry', 'ekf_filter.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',      default_value='false',             description=''),
        DeclareLaunchArgument('use_ekf',           default_value='true',              description='Run EKF'),
        DeclareLaunchArgument('use_ukf',           default_value='false',             description='Run UKF'),
        DeclareLaunchArgument('ekf_params_file',   default_value='',                  description='Path to EKF config'),
        DeclareLaunchArgument('ukf_params_file',   default_value='',                  description='Path to UKF config'),
        DeclareLaunchArgument('localization_ns',   default_value='localization',      description='Localization node namespace'),
        DeclareLaunchArgument('localization_mode', default_value='110',               description='Localization Mode (Binary: LiDAR-Cam0-Cam1, e.g. 110)'),
        DeclareLaunchArgument('camera_00_ns', default_value='camera_00', description='Primary camera sensor namespace'),
        DeclareLaunchArgument('camera_01_ns', default_value='camera_01', description='Secondary camera sensor namespace'),
        DeclareLaunchArgument('lidar_ns',          default_value='lidar_00',          description='Lidar sensor namespace'),
        DeclareLaunchArgument('launch_aruco_00', default_value='false', description='Launch ArUco odometry updater 00'),
        DeclareLaunchArgument('launch_aruco_01', default_value='false', description='Launch ArUco odometry updater 01'),
        DeclareLaunchArgument('launch_aruco_02', default_value='false', description='Launch ArUco odometry updater 02'),
        OpaqueFunction(function=launch_setup)
    ])
