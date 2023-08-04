from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    namespace_param = LaunchConfiguration('namespace')

    # Acquire the driver param file
    config_file_declaration = DeclareLaunchArgument('config_file',
                                                    default_value=os.path.join(
                                                        get_package_share_directory('bluespace_ai_xsens_mti_driver'),
                                                        'param',
                                                        'xsens_mti_node.yaml'))

    params_namespace_declare = DeclareLaunchArgument('namespace',
                                                     default_value='/xsens',
                                                     description='Driver namespace')

    xsens_driver_node = Node(package='bluespace_ai_xsens_mti_driver',
                           namespace=[namespace_param],
                           executable='xsens_mti_node',
                           output='screen',
                           parameters=[config_file])

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        config_file_declaration,
        params_namespace_declare,
        xsens_driver_node,
    ])
