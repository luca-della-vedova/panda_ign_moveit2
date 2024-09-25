#!/usr/bin/env -S ros2 launch
"""Example of planning with MoveIt2 and executing motions using ROS 2 controllers within Gazebo"""

from os import path
from pathlib import Path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    world = LaunchConfiguration("world")
    name = LaunchConfiguration("name")
    prefix = LaunchConfiguration("prefix")
    collision_arm = LaunchConfiguration("collision_arm")
    collision_gripper = LaunchConfiguration("collision_gripper")
    ros2_control_command_interface = LaunchConfiguration(
        "ros2_control_command_interface"
    )
    gazebo_preserve_fixed_joint = LaunchConfiguration("gazebo_preserve_fixed_joint")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_verbosity = LaunchConfiguration("gz_verbosity")
    log_level = LaunchConfiguration("log_level")
    panda_x = LaunchConfiguration("panda_x")
    panda_y = LaunchConfiguration("panda_y")
    panda_z = LaunchConfiguration("panda_z")
    panda_r = LaunchConfiguration("panda_r")
    panda_p = LaunchConfiguration("panda_p")
    panda_Y = LaunchConfiguration("panda_Y")

    # List of processes to be executed
    xacro_path = Path(get_package_share_directory('panda_description'))/"urdf"/"panda.urdf.xacro"
    urdf_string = Command(
        [
            "xacro",
            f" {xacro_path}",
            " name:=", name,
            " prefix:=", prefix,
            " collision_arm:=", collision_arm,
            " collision_gripper:=", collision_gripper,
            " ros2_control:=", "true",
            " ros2_control_plugin:=", "gz",
            " ros2_control_command_interface:=", ros2_control_command_interface,
            " gazebo_preserve_fixed_joint:=", gazebo_preserve_fixed_joint,
        ]
    )
    # List of included launch descriptions
    launch_descriptions = [
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ("gz_args", [world, " -r -v ", gz_verbosity])
            ],
        ),
        # Launch move_group of MoveIt 2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("panda_moveit_config"),
                        "launch",
                        "move_group.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ("name:=", name),
                ("prefix:=", prefix),
                ("collision_arm", collision_arm),
                ("collision_gripper", collision_gripper),
                ("ros2_control", "true"),
                ("ros2_control_plugin", "gz"),
                ("ros2_control_interface", ros2_control_command_interface),
                (
                    "gazebo_preserve_fixed_joint",
                    gazebo_preserve_fixed_joint,
                ),
                ("rviz_config", rviz_config),
                ("use_sim_time", use_sim_time),
                ("log_level", log_level),
            ],
        ),
    ]

    # List of nodes to be launched
    nodes = [
        # ros_gz_sim_create
        Node(
            package="ros_gz_sim",
            executable="create",
            output="log",
            arguments=[
                "-string", urdf_string,
                "-x",
                panda_x,
                "-y",
                panda_y,
                "-z",
                panda_z,
                "-R",
                panda_r,
                "-P",
                panda_p,
                "-Y",
                panda_Y,
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # ros_gz_bridge (clock -> ROS 2)
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="log",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    return LaunchDescription(
        declared_arguments + launch_descriptions + nodes
    )


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # SDF world for Gazebo
        DeclareLaunchArgument(
            "world",
            default_value="default.sdf",
            description="Name or filepath of the Gazebo world to load.",
        ),
        # Naming of the robot
        DeclareLaunchArgument(
            "name",
            default_value="panda",
            description="Name of the robot.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value=[LaunchConfiguration("name"), "_"],
            description="Prefix for all robot entities. If modified, then joint names in the configuration of controllers must also be updated.",
        ),
        # Collision geometry
        DeclareLaunchArgument(
            "collision_arm",
            default_value="true",
            description="Flag to enable collision geometry for the arm.",
        ),
        DeclareLaunchArgument(
            "collision_gripper",
            default_value="true",
            description="Flag to enable collision geometry for the gripper.",
        ),
        # ROS 2 control
        DeclareLaunchArgument(
            "ros2_control_command_interface",
            default_value="position",
            description="The output control command interface provided by ros2_control ('position', 'velocity', 'effort' or certain combinations 'position,velocity').",
        ),
        # Gazebo
        DeclareLaunchArgument(
            "gazebo_preserve_fixed_joint",
            default_value="false",
            description="Flag to preserve fixed joints and prevent lumping when generating SDF for Gazebo.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("panda_moveit_config"),
                "rviz",
                "moveit.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "gz_verbosity",
            default_value="3",
            description="Verbosity level for Gazebo (0~4).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
        DeclareLaunchArgument(
            "panda_x",
            default_value="0",
            description="X component of arm spawning position in meters",
        ),
        DeclareLaunchArgument(
            "panda_y",
            default_value="0",
            description="Y component of arm spawning position in meters",
        ),
        DeclareLaunchArgument(
            "panda_z",
            default_value="0",
            description="Z component of arm spawning position in meters",
        ),
        DeclareLaunchArgument(
            "panda_r",
            default_value="0",
            description="Roll component of arm spawning orientation in radians",
        ),
        DeclareLaunchArgument(
            "panda_p",
            default_value="0",
            description="Pitch component of arm spawning orientation in radians",
        ),
        DeclareLaunchArgument(
            "panda_Y",
            default_value="0",
            description="Yaw component of arm spawning orientation in radians",
        ),
    ]
