from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Decode pipeline overview
# ────────────────────────────────────────────────────────────────
# enable_decode=false (default):
#   └─ enable_republish=true  →  image_transport republish (single-threaded)
#
# enable_decode=true + decode_backend=cpu:
#   └─ cpu_image_decoder    (ThreadPoolExecutor + libjpeg-turbo)
#
# enable_decode=true + decode_backend=gpu:
#   └─ decode_gpu_vendor=nvidia  →  gpu_nvidia_decoder  (nvjpeg → cv2.cuda → CPU)
#   └─ decode_gpu_vendor=amd     →  gpu_amd_decoder     (rocJPEG → PyOpenCL → CPU)
#   └─ decode_gpu_vendor=auto    →  probed at launch via /dev & /opt/rocm
#
# All paths publish to:  <camera_ns>/rgb/image_raw/uncompressed
# Stereo depth is already raw — no decoder needed.
# ────────────────────────────────────────────────────────────────

def launch_setup(context, *args, **kwargs):
    camera_ns        = LaunchConfiguration('camera_ns').perform(context)
    camera_name      = LaunchConfiguration('camera_name').perform(context)
    params_file      = LaunchConfiguration('params_file').perform(context)
    enable_composable = LaunchConfiguration('enable_composable')
    enable_republish  = LaunchConfiguration('enable_republish')

    # Decode arguments
    enable_decode      = LaunchConfiguration('enable_decode').perform(context).lower()
    decode_backend     = LaunchConfiguration('decode_backend').perform(context).lower()
    decode_threads     = LaunchConfiguration('decode_threads').perform(context)
    decode_queue_size  = LaunchConfiguration('decode_queue_size').perform(context)
    decode_gpu_vendor  = LaunchConfiguration('decode_gpu_vendor').perform(context).lower()

    camera_base_frame = f'{camera_name}_link'
    camera_parent_frame = f'sensor_mount_{camera_name[-2:]}_link'

    camera_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'),
                ' "',
                PathJoinSubstitution([
                    FindPackageShare("playground"),
                    'urdf',
                    'depthai_camera_description.urdf.xacro'
                ]),
                '"',
                f" camera_name:={camera_name}",
                f" base_frame:={camera_base_frame}",
                f" parent_frame:={camera_parent_frame}",
            ]),
            value_type=str
        )
    }

    camera_nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name=f"{camera_name}_state_publisher",
            namespace=camera_ns,
            parameters=[camera_description],
            condition=UnlessCondition(enable_composable)
        ),
        Node(
            package="depthai_ros_driver",
            executable="camera_node",
            name=camera_name,
            namespace=camera_ns,
            parameters=[params_file],
            condition=UnlessCondition(enable_composable)
        ),
    ]

    composable_nodes = [
        ComposableNode(
            package="robot_state_publisher",
            plugin="robot_state_publisher::RobotStatePublisher",
            name=f"{camera_name}_state_publisher",
            namespace=camera_ns,
            parameters=[camera_description],
        ),
        ComposableNode(
            package="depthai_ros_driver",
            plugin="depthai_ros_driver::Camera",
            name=camera_name,
            namespace=camera_ns,
            parameters=[params_file],
        ),
    ]
    
    container = ComposableNodeContainer(
        name=f'{camera_name}_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='screen',
        condition=IfCondition(enable_composable)
    )

    republish_nodes = [
        Node(
            package="image_transport",
            executable="republish",
            name="rgb_republish",
            namespace=camera_ns,
            arguments=['compressed', 'raw'],
            remappings=[
                ("in/compressed", f"{camera_ns}/rgb/image_raw/compressed"),
                ("out", f"{camera_ns}/rgb/image_raw/uncompressed")
            ],
            condition=IfCondition(enable_republish)
        ),
        Node(
            package="image_transport",
            executable="republish",
            name="stereo_republish",
            namespace=camera_ns,
            arguments=['compressed', 'raw'],
            remappings=[
                ("in/compressed", f"{camera_ns}/stereo/image_raw/compressed"),
                ("out", f"{camera_ns}/stereo/image_raw/uncompressed")
            ],
            condition=IfCondition(enable_republish)
        ),
    ]

    # ----------------------------------------------------------------
    # Decoder nodes  (enable_decode=true replaces the republish path)
    # ----------------------------------------------------------------
    decode_nodes = []
    if enable_decode == 'true':
        # Common remapping: compressed in → uncompressed out
        decode_remappings = [
            ('in/compressed', f'{camera_ns}/rgb/image_raw/compressed'),
            ('out',           f'{camera_ns}/rgb/image_raw/uncompressed'),
        ]

        if decode_backend == 'gpu':
            import os  # noqa: PLC0415

            # Resolve 'auto' vendor at launch time using lightweight OS probes.
            # No GPU libraries are imported here — just path / device-node checks.
            resolved_vendor = decode_gpu_vendor
            if resolved_vendor == 'auto':
                _has_nvidia = (
                    os.path.exists('/dev/nvidiactl')
                    or os.path.exists('/proc/driver/nvidia')
                )
                _has_amd = (
                    os.path.isdir('/opt/rocm')
                    or os.path.exists('/dev/kfd')
                )
                if _has_nvidia:
                    resolved_vendor = 'nvidia'
                elif _has_amd:
                    resolved_vendor = 'amd'
                else:
                    # No GPU markers found — fall through to CPU decoder
                    resolved_vendor = 'cpu'

            if resolved_vendor == 'nvidia':
                decode_nodes.append(
                    Node(
                        package='playground',
                        executable='gpu_nvidia_decoder',
                        name='rgb_gpu_decoder',
                        namespace=camera_ns,
                        remappings=decode_remappings,
                        parameters=[{
                            'gpu_device_id': 0,
                            'queue_size':    int(decode_queue_size),
                            'encoding':      'bgr8',
                        }],
                        output='screen',
                    )
                )
            elif resolved_vendor == 'amd':
                decode_nodes.append(
                    Node(
                        package='playground',
                        executable='gpu_amd_decoder',
                        name='rgb_gpu_decoder',
                        namespace=camera_ns,
                        remappings=decode_remappings,
                        parameters=[{
                            'gpu_device_id': 0,
                            'queue_size':    int(decode_queue_size),
                            'encoding':      'bgr8',
                        }],
                        output='screen',
                    )
                )
            else:
                # auto-detected no GPU — spawn CPU decoder as graceful fallback
                decode_nodes.append(
                    Node(
                        package='playground',
                        executable='cpu_image_decoder',
                        name='rgb_cpu_decoder',
                        namespace=camera_ns,
                        remappings=decode_remappings,
                        parameters=[{
                            'num_threads': int(decode_threads),
                            'queue_size':  int(decode_queue_size),
                            'encoding':    'bgr8',
                        }],
                        output='screen',
                    )
                )

        else:  # cpu (default or explicit)
            decode_nodes.append(
                Node(
                    package='playground',
                    executable='cpu_image_decoder',
                    name='rgb_cpu_decoder',
                    namespace=camera_ns,
                    remappings=decode_remappings,
                    parameters=[{
                        'num_threads': int(decode_threads),
                        'queue_size':  int(decode_queue_size),
                        'encoding':    'bgr8',
                    }],
                    output='screen',
                )
            )

    return camera_nodes + [container] + republish_nodes + decode_nodes

def generate_launch_description():
    return LaunchDescription([
        # Core camera arguments
        DeclareLaunchArgument('camera_ns',          default_value='camera_00'),
        DeclareLaunchArgument('camera_name',         default_value='camera_00'),
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('enable_composable',   default_value='false'),
        DeclareLaunchArgument('enable_republish',    default_value='false'),

        # Multithreaded decode arguments
        # These are populated by bringup.launch.py via bringup_profile.yaml.
        # They can also be overridden directly from the CLI.
        DeclareLaunchArgument(
            'enable_decode',
            default_value='false',
            description='Enable multithreaded decoder node for RGB stream '
                        '(replaces single-threaded image_transport republish)'
        ),
        DeclareLaunchArgument(
            'decode_backend',
            default_value='cpu',
            description='Decoder backend: cpu | gpu'
        ),
        DeclareLaunchArgument(
            'decode_threads',
            default_value='4',
            description='ThreadPoolExecutor worker count (cpu backend only)'
        ),
        DeclareLaunchArgument(
            'decode_queue_size',
            default_value='5',
            description='Subscriber / publisher queue depth for decoder node'
        ),
        DeclareLaunchArgument(
            'decode_gpu_vendor',
            default_value='auto',
            description='GPU vendor for decoder: auto | nvidia | amd (gpu backend only)'
        ),

        OpaqueFunction(function=launch_setup)
    ])
