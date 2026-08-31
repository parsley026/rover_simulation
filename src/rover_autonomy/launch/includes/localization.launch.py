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
    localization_mode = int(LaunchConfiguration('localization_mode').perform(context))
    lidar_ns = LaunchConfiguration('lidar_ns').perform(context)
    cam0_ns = LaunchConfiguration('camera_00_ns').perform(context)
    cam1_ns = LaunchConfiguration('camera_01_ns').perform(context)

    # Config for the primary absolute reference (Pose: x, y, z, r, p, y)
    odom_pose_config = [True,  True,  True,
                        True,  True,  True,
                        False, False, False,
                        False, False, False,
                        False, False, False]

    # Config for secondary sensors (Velocity: vx, vy, vz, vr, vp, vy)
    odom_twist_config = [False, False, False,
                         False, False, False,
                         True,  True,  True,
                         True,  True,  True,
                         False, False, False]

    # Config for IMU (Angular Velocity only)
    imu_config = [False, False, False,
                  False, False, False,
                  False, False, False,
                  True,  True,  True,
                  False, False, False]

    mode_params = {}

    if localization_mode == 1:
        # 1: Full Fusion (LiDAR Pose + Cam0 Velocity + Cam1 Velocity)
        mode_params = {
            'odom0': f"/{lidar_ns}/odom",  'odom0_config': odom_pose_config,  'odom0_relative': True,
            'odom1': f"/{cam0_ns}/odom",   'odom1_config': odom_twist_config, 'odom1_relative': False,
            'odom2': f"/{cam1_ns}/odom",   'odom2_config': odom_twist_config, 'odom2_relative': False,
            'imu0': f"/{lidar_ns}/imu",    'imu0_config': imu_config, 'imu0_queue_size': 50, 'imu0_relative': False, 'imu0_remove_gravitational_acceleration': True,
        }
    elif localization_mode == 2:
        # 2: LiDAR + Cam0 (LiDAR Pose + Cam0 Velocity)
        mode_params = {
            'odom0': f"/{lidar_ns}/odom",  'odom0_config': odom_pose_config,  'odom0_relative': True,
            'odom1': f"/{cam0_ns}/odom",   'odom1_config': odom_twist_config, 'odom1_relative': False,
            'imu0': f"/{lidar_ns}/imu",    'imu0_config': imu_config, 'imu0_queue_size': 50, 'imu0_relative': False, 'imu0_remove_gravitational_acceleration': True,
        }
    elif localization_mode == 3:
        # 3: Dual Camera (Cam0 Pose + Cam1 Velocity)
        mode_params = {
            'odom0': f"/{cam0_ns}/odom",   'odom0_config': odom_pose_config,  'odom0_relative': True,
            'odom1': f"/{cam1_ns}/odom",   'odom1_config': odom_twist_config, 'odom1_relative': False,
        }
    elif localization_mode == 4:
        # 4: Cam0 Only (Cam0 Pose)
        mode_params = {
            'odom0': f"/{cam0_ns}/odom",   'odom0_config': odom_pose_config,  'odom0_relative': True,
        }
    elif localization_mode == 5:
        # 5: LiDAR Only (LiDAR Pose)
        mode_params = {
            'odom0': f"/{lidar_ns}/odom",  'odom0_config': odom_pose_config,  'odom0_relative': True,
            'imu0': f"/{lidar_ns}/imu",    'imu0_config': imu_config, 'imu0_queue_size': 50, 'imu0_relative': False, 'imu0_remove_gravitational_acceleration': True,
        }

    # Add standard queue sizes dynamically
    for key in list(mode_params.keys()):
        if 'config' in key:
            base_name = key.replace('_config', '')
            if f"{base_name}_queue_size" not in mode_params:
                mode_params[f"{base_name}_queue_size"] = 10
            if f"{base_name}_differential" not in mode_params:
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
        DeclareLaunchArgument('localization_mode', default_value='1',                 description='Localization Mode (1-5)'),
        DeclareLaunchArgument('camera_00_ns', default_value='camera_00', description='Primary camera sensor namespace'),
        DeclareLaunchArgument('camera_01_ns', default_value='camera_01', description='Secondary camera sensor namespace'),
        DeclareLaunchArgument('lidar_ns',          default_value='lidar_00',          description='Lidar sensor namespace'),
        OpaqueFunction(function=launch_setup)
    ])
