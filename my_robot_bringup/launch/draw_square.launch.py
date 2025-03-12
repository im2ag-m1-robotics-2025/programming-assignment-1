from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  #arguments and parameters
  side_length_arg = DeclareLaunchArgument(
    'side_length',
    default_value='2.0',
    description='Side length of the square to be drawn'
  )
  
  velocity_arg = DeclareLaunchArgument(
    'velocity',
    default_value='0.5',
    description='Linear velocity for drawing the square'
  )
  
  turtlesim_node = Node(
    package='turtlesim',
    executable='turtlesim_node',
    name='turtlesim',
    output='screen'
  )
  
  #launching the draw_square node
  draw_square_node = Node(
    package='my_py_pkg',
    executable='draw_square',
    name='draw_square',
    output='screen',
    parameters=[{
      'default_side_length': LaunchConfiguration('side_length'),
      'default_velocity': LaunchConfiguration('velocity')
      }]
  )
  
  return LaunchDescription([
    side_length_arg,
    velocity_arg,
    turtlesim_node,
    draw_square_node
  ])