from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap, Node


def launch_setup(context, *args, **kwargs):
    # Define package directories
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_roomba_nav2 = get_package_share_directory('mini_navigate')

    # Define launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    rviz_config_file = LaunchConfiguration('rviz_config')

    # Define remapping for laser scan topics
    namespace = LaunchConfiguration('namespace').perform(context)
    if namespace and not namespace.startswith('/'):
        namespace = '/' + namespace

    # Include the main navigation launch file
    launch_nav2 = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'bringup_launch.py'])

    nav2 = GroupAction([
        PushRosNamespace(namespace),
        SetRemap(namespace + '/global_costmap/scan', namespace + '/scan'),
        SetRemap(namespace + '/local_costmap/scan', namespace + '/scan'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_nav2),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'namespace': namespace,
                'use_composition': 'False'
            }.items(),
        ),
    ])

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return [nav2, rviz_node]


def generate_launch_description():
    # Declare launch arguments
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                get_package_share_directory('mini_navigate'),
                'param',
                'nav2_params.yaml'
            ]),
            description='Full path to the ROS2 parameters file to use'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                get_package_share_directory('mini_navigate'),
                'map',
                'mi_mapa_save.yaml'
            ]),
            description='Full path to map file to load'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'rviz',
                'nav2_default_view.rviz'
            ]),
            description='Full path to the RVIZ config file to use'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace'
        ),
        OpaqueFunction(function=launch_setup)
    ])
