#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():

    remappable_topics = [
        DeclareLaunchArgument("input_topic", default_value="~/input"),
        DeclareLaunchArgument("output_topic", default_value="~/output"),
    ]

    args = [
        DeclareLaunchArgument("name", default_value="nighthawk_ros", description="node name"),
        DeclareLaunchArgument("namespace", default_value="", description="node namespace"),
        DeclareLaunchArgument("params", default_value=os.path.join(get_package_share_directory("nighthawk_ros"), "config", "params.yml"), description="path to parameter file"),
        DeclareLaunchArgument("log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="use simulation clock"),
        *remappable_topics,
    ]

    nodes = [
        Node(
            package="nighthawk_ros",
            executable="nighthawk_ros",
            namespace=LaunchConfiguration("namespace"),
            name=LaunchConfiguration("name"),
            parameters=[LaunchConfiguration("params")],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            remappings=[(la.default_value[0].text, LaunchConfiguration(la.name)) for la in remappable_topics],
            output="screen",
            emulate_tty=True,
        ),

        # Node(
        #     package="nighthawk_ros",
        #     executable="nighthawk_score_node",
        #     namespace=LaunchConfiguration("namespace"),
        #     name="nighthawk_score",
        #     parameters=[LaunchConfiguration("params")],
        #     output="screen",
        # ),
        
        # Node(
        #     package="nighthawk_ros",
        #     executable="light_control_ros",
        #     namespace=LaunchConfiguration("namespace"),
        #     name="light_control",
        #     parameters=[LaunchConfiguration("params")],
        #     output="screen",
        # ),

        Node(
            package="nighthawk_ros",
            executable="image_writer",
            namespace=LaunchConfiguration("namespace"),
            name="image_writer",
            parameters=[LaunchConfiguration("params")],
            output="screen",
        ),
    ]

    return LaunchDescription([
        *args,
        SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
        *nodes,
    ])