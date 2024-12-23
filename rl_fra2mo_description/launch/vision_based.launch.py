from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    fra2mo_dir = FindPackageShare('rl_fra2mo_description')
    aruco_dir = FindPackageShare('aruco_ros')
    
    # Define paths to the launch files we want to include
    gazebo_launch = PathJoinSubstitution(
        [fra2mo_dir, 'launch', 'centermap.launch.py']
    )
    
    explore_launch = PathJoinSubstitution(
        [fra2mo_dir, 'launch', 'fra2mo_explore.launch.py']
    )
    
    # Include the Gazebo simulation launch
    gazebo_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch])
    )
    
    # Include the Navigation launch
    explore_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([explore_launch]),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )
    
    # Configuration for ArUco marker detection using aruco_ros
    aruco_single_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        output='screen',
        parameters=[{
            'marker_id': 115,  # ID del marker da rilevare
            'marker_size': 0.2,  # Dimensione del marker in metri
            'reference_frame': 'map',  # Frame di riferimento
            'marker_frame': 'aruco_marker_frame',  # Frame del marker
            'camera_frame': 'camera_link_optical',  # Frame della telecamera
      #        'use_sim_time': True
        }],
        remappings=[
            ('/image', '/camera'),  # Remapping del topic dell'immagine
            ('/camera_info', '/camera_info')  # Remapping del topic delle informazioni della camera
        ]
    )

    static_aruco_tf2_node = Node(
        package='rl_fra2mo_description',
        executable='static_aruco_tf2_broadcaster',
        name='static_aruco_tf2_broadcaster',
        output='screen'
    )
    
    # Create list of nodes to launch
    launches_to_start = [
        gazebo_launch_include,
        explore_launch_include,
        aruco_single_node,  # ArUco detection node
        static_aruco_tf2_node,
    ]
    
    return LaunchDescription(launches_to_start)
