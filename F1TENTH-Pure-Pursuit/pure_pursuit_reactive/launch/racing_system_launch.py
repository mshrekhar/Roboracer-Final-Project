from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # --- 1. DECLARE ARGUMENTS ---
    
    # Matching your CLI usage: change 'track' to 'waypoint_file'
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file', 
        default_value='waypoints.csv',
        description='Full path to the waypoints CSV file'
    )

    # Racing Params
    max_speed_arg = DeclareLaunchArgument('max_speed', default_value='5.0')
    min_speed_arg = DeclareLaunchArgument('min_speed', default_value='2.0')
    brake_gain_arg = DeclareLaunchArgument('brake_gain', default_value='1.8')
    
    # Lookahead Params
    min_la_arg = DeclareLaunchArgument('min_la', default_value='0.7')
    max_la_arg = DeclareLaunchArgument('max_la', default_value='3.8')
    la_gain_arg = DeclareLaunchArgument('la_gain', default_value='0.3')
    
    # VO Safety Params
    vo_margin_arg = DeclareLaunchArgument('margin', default_value='0.15')

    # --- 2. DEFINE THE NODES ---

    pure_pursuit_node = Node(
        package='pure_pursuit_reactive',
        executable='pure_pursuit_node',
        name='pure_pursuit',
        # Ensure the node uses the parameters we are passing
        parameters=[{
            'max_speed': LaunchConfiguration('max_speed'),
            'min_speed': LaunchConfiguration('min_speed'),
            'brake_gain': LaunchConfiguration('brake_gain'),
            'min_lookahead': LaunchConfiguration('min_la'),
            'max_lookahead': LaunchConfiguration('max_la'),
            'lookahead_gain': LaunchConfiguration('la_gain'),
            'waypoint_file': LaunchConfiguration('waypoint_file'), # Matches the Argument
            'wheelbase': 0.33,
            'max_steer_angle': 0.4189
        }],
        output='screen'
    )

    vo_node = Node(
        package='pure_pursuit_reactive',
        executable='vo_filter_node',
        name='vo_filter',
        parameters=[{
            'safety_margin': LaunchConfiguration('margin'),
            'robot_radius': 0.33,
            'time_horizon': 1.5
        }],
        output='screen'
    )

    return LaunchDescription([
        waypoint_file_arg,
        max_speed_arg,
        min_speed_arg,
        brake_gain_arg,
        min_la_arg,
        max_la_arg,
        la_gain_arg,
        vo_margin_arg,
        pure_pursuit_node,
        vo_node
    ])