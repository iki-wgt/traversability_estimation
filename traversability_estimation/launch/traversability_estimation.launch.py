from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from launch.conditions import LaunchConfigurationEquals
import os


def generate_launch_description():
    # Declare the launch arguments to allow topic remapping

    # Declare argument for parameters file, with default value pointing to package's params file
    # default_params_file = PathJoinSubstitution([
    #     get_package_share_directory('traversability_estimation'),
    #     'config',
    #     'traversability_estimation_params.yaml'
    # ])
    default_params_file = "/root/ros2_ws/src/traversability_estimation/traversability_estimation/config/params.yaml"
    rviz_config_file = "/root/ros2_ws/src/traversability_estimation/traversability_estimation/config/traversability.rviz"

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the parameters file to use'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch rviz2 or not'
    )

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace of the robot'
    )

    declare_tf_topic = DeclareLaunchArgument(
        'tf_topic',
        default_value='/tf',
        description='Topic for TF'
    )

    declare_tf_static_topic = DeclareLaunchArgument(
        'tf_static_topic',
        default_value='/tf_static',
        description='Topic for TF static'
    )

    # Create the node, including the parameters and remappings
    traversability_estimation_node = Node(
        package='traversability_estimation',
        executable='traversability_estimation_node',
        name='traversability_estimation',
        namespace=LaunchConfiguration('namespace'),
        remappings=[('/tf', LaunchConfiguration('tf_topic')), ('/tf_static', LaunchConfiguration('tf_static_topic'))],
        parameters=[LaunchConfiguration('params_file')],
    )

    graph_planning_node = Node(
        package='traversability_estimation',
        executable='graph_planning_node',
        name='graph_planning',
        namespace=LaunchConfiguration('namespace'),
        remappings=[('/tf', LaunchConfiguration('tf_topic')), ('/tf_static', LaunchConfiguration('tf_static_topic'))],
        parameters=[LaunchConfiguration('params_file')],
    )

    graph_visualization_node = Node(
        package='traversability_estimation',
        executable='graph_visualization_node',
        name='graph_visualization',
        namespace=LaunchConfiguration('namespace'),
        remappings=[('/tf', LaunchConfiguration('tf_topic')), ('/tf_static', LaunchConfiguration('tf_static_topic'))],
        parameters=[LaunchConfiguration('params_file')],
    )


    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        namespace=LaunchConfiguration('namespace'),
        remappings=[('/tf', LaunchConfiguration('tf_topic')), ('/tf_static', LaunchConfiguration('tf_static_topic'))],
        condition=LaunchConfigurationEquals('use_rviz', 'true'),
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        declare_params_file,
        declare_use_rviz,
        declare_namespace,
        declare_tf_topic,
        declare_tf_static_topic,
        traversability_estimation_node,
        graph_planning_node,
        graph_visualization_node,
        rviz_node
    ])
