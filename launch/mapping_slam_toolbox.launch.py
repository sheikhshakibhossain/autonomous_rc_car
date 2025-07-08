from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('autonomous_rc_car')
    urdf_path = os.path.join(pkg_path, 'urdf', 'autonomous_rc_car.urdf')

    return LaunchDescription([

        # Robot description TFs
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_path).read()
            }]
        ),

        # Fixed wheel joint states
        Node(
            package='autonomous_rc_car',
            executable='fixed_joint_publisher',
            name='fixed_joint_publisher',
            output='screen',
        ),

        # Hybrid odometry: right wheel encoder + /cmd_vel
        Node(
            package='autonomous_rc_car',
            executable='hybrid_odometry_publisher',
            name='hybrid_odometry_publisher',
            output='screen'
        ),

        # Static TF: base_link → lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_tf',
            arguments=['0', '0', '0.13', '0', '0', '0', 'base_link', 'lidar'],
            output='screen'
        ),

        # RPLidar node
        Node(
	    package='rplidar_ros',
	    executable='rplidar_node',
	    name='rplidar_node',
	    output='screen',
	    parameters=[{
		'channel_type': 'serial',
		'serial_port': '/dev/ttyUSB0',
		'serial_baudrate': 460800,
		'frame_id': 'lidar',
		'inverted': False,
		'angle_compensate': True,
		'scan_mode': 'Standard',
		'publish_tf': False  # 🔧 disables conflicting static TFs
	    }]
	),

        # SLAM Toolbox (start after TFs/odom available)
        TimerAction(
	    period=2.0,
	    actions=[
		Node(
		    package='slam_toolbox',
		    executable='sync_slam_toolbox_node',
		    name='slam_toolbox',
		    output='screen',
		    parameters=[{
		        'use_sim_time': False,
		        'transform_timeout': 0.3,
		        'odom_frame': 'odom',
		        'base_frame': 'base_link',
		        'scan_topic': 'scan',
		        'laser_min_range': 0.2,
		        'laser_max_range': 16.0
		    }]
		)
	    ]
	)

    ])

