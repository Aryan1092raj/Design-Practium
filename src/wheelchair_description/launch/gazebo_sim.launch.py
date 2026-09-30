#!/usr/bin/env python3
"""
Full Gazebo simulation bring-up: gazebo_launch.py + ros2_control spawners + RViz.

gazebo_launch.py starts Gazebo and the robot, but it does NOT start the
ros2_control controllers. Without them the DiffDriveController is never
activated, so /wc_control/odom stays silent and the wheelchair never moves.

This wrapper includes gazebo_launch.py unmodified and adds the missing pieces.

Usage:
    ros2 launch wheelchair_description gazebo_sim.launch.py
    ros2 launch wheelchair_description gazebo_sim.launch.py world_name:=small_house
    ros2 launch wheelchair_description gazebo_sim.launch.py use_rviz:=false

    # Drive it yourself: bridge /cmd_vel (Twist) -> /wc_control/cmd_vel (TwistStamped)
    ros2 launch wheelchair_description gazebo_sim.launch.py teleop:=true
    # ...and additionally open a keyboard window in its own terminal
    ros2 launch wheelchair_description gazebo_sim.launch.py teleop:=true teleop_keyboard:=true

    The LiDAR is bridged to /scan by default (bridge_lidar:=false to disable), so RViz
    draws the walls/furniture of the world while you drive around.

    Localization: the EKF is started by default (ekf:=false to disable). It fuses
    /wc_control/odom + the sim IMU (republished to /imu) and publishes the
    odom -> base_link transform that wc_control deliberately does not
    (enable_odom_tf: false, because the real robot lets the EKF own that TF).
    Without it, SLAM/Nav2 cannot run in simulation.

    Mapping: slam:=true adds slam_toolbox (map -> odom + /map). It reuses one of the
    real-robot configs from wheelchair_localization, with simulation values layered on
    top from config/slam_sim.yaml. Drive around with teleop:=true and the map builds.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_description = get_package_share_directory("wheelchair_description")
    # Reuse the EKF tuning that the real robot uses (wheelchair_bringup includes the
    # same file), so simulation and hardware share one filter configuration.
    pkg_localization = get_package_share_directory("wheelchair_localization")
    ekf_config = os.path.join(pkg_localization, "config", "ekf.yaml")

    world_name = LaunchConfiguration("world_name")
    use_rviz = LaunchConfiguration("use_rviz")
    teleop = LaunchConfiguration("teleop")
    teleop_keyboard = LaunchConfiguration("teleop_keyboard")
    bridge_lidar = LaunchConfiguration("bridge_lidar")
    ekf = LaunchConfiguration("ekf")
    slam = LaunchConfiguration("slam")
    slam_params_file = LaunchConfiguration("slam_params_file")

    # SLAM needs odom -> base_link (the EKF) plus the bridged /scan.
    slam_ready = PythonExpression([
        "'", slam, "' == 'true' and '", ekf, "' == 'true'",
    ])

    declare_world_name = DeclareLaunchArgument(
        "world_name", default_value="empty",
        description="World file name (without .world) from wheelchair_description/worlds"
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="Start RViz2 with the simulation config"
    )
    declare_teleop = DeclareLaunchArgument(
        "teleop", default_value="false",
        description="Bridge /cmd_vel (Twist) to /wc_control/cmd_vel (TwistStamped) so any "
                    "teleop tool can drive the wheelchair"
    )
    declare_teleop_keyboard = DeclareLaunchArgument(
        "teleop_keyboard", default_value="false",
        description="Also start teleop_twist_keyboard in its own terminal window "
                    "(requires teleop:=true)"
    )
    declare_bridge_lidar = DeclareLaunchArgument(
        "bridge_lidar", default_value="true",
        description="Bridge the Gazebo GPU LiDAR to /scan (sensor_msgs/LaserScan)"
    )
    declare_ekf = DeclareLaunchArgument(
        "ekf", default_value="true",
        description="Run robot_localization EKF (odom -> base_link TF) + the IMU "
                    "republisher it needs. Required for SLAM/Nav2 in simulation."
    )
    declare_slam = DeclareLaunchArgument(
        "slam", default_value="false",
        description="Run slam_toolbox to map the world while driving (requires ekf:=true "
                    "and bridge_lidar:=true). Publishes /map and the map -> odom TF."
    )
    declare_slam_params = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(pkg_localization, "config",
                                   "slam_toolbox_motion_compensated_v2.yaml"),
        description="Real-robot slam_toolbox config to reuse (this is the same LiDAR-only "
                    "default that wheelchair_bringup/wheelchair_slam_mapping.launch.py "
                    "uses); the sim overrides in config/slam_sim.yaml are applied on top"
    )

    # Gazebo, robot_state_publisher, spawn entity and gz-ROS bridge
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_description, "launch", "gazebo_launch.py")),
        launch_arguments={"world_name": world_name}.items(),
    )

    # The controller manager is created by the gz_ros2_control plugin *inside*
    # Gazebo, and only after the robot has been spawned. Spawners block until it
    # answers, but the delay keeps the start-up log readable.
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager-timeout", "90"],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ],
    )

    wc_control_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["wc_control", "--controller-manager-timeout", "90"],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg_description, "rviz", "sim_view.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    # diff_drive_controller runs under the /wc_control namespace and use_stamped_vel
    # is true, so it expects geometry_msgs/msg/TwistStamped on /wc_control/cmd_vel.
    # The plain Twist teleop output has to be converted first. This bridge lets ANY
    # teleop source (keyboard, joystick, your own node) publish plain Twist on /cmd_vel.
    teleop_converter = Node(
        package="scripts",
        executable="twist_stamped_teleop",
        name="twist_stamped_teleop",
        output="screen",
        parameters=[{"use_sim_time": True}],
        remappings=[
            ("cmd_vel_in", "cmd_vel"),
            ("cmd_vel_out", "wc_control/cmd_vel"),
        ],
        condition=IfCondition(teleop),
    )

    # teleop_twist_keyboard reads single keystrokes from its own TTY, so it needs a
    # terminal window of its own. x-terminal-emulator is the Ubuntu wrapper (ghostty
    # here); -e keeps the terminal attached to the command so Ctrl-C still stops it.
    # NOTE: prefix must be ONE string, not a list - launch_ros joins list entries with
    # '-' into a single bogus command name (e.g. "x-terminal-emulator-e").
    keyboard_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="x-terminal-emulator -e",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(PythonExpression([
            "'", teleop, "' == 'true' and '", teleop_keyboard, "' == 'true'",
        ])),
    )

    # The URDF publishes the GPU LiDAR on the fixed gz topic /scan with frame_id
    # "lidar" (see urdf/wc_gazebo.xacro), so one fixed bridge string works in every
    # world file regardless of the world name.
    lidar_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="lidar_bridge",
        arguments=["/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(bridge_lidar),
    )

    # The EKF's imu0 input is /imu (hardware name), while Gazebo publishes /imu/out.
    # scripts/imu_out_to_imu republishes it with frame_id base_link, exactly as it does
    # for the real robot.
    imu_republisher = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="scripts",
                executable="imu_out_to_imu",
                name="imu_out_to_imu",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ],
        condition=IfCondition(ekf),
    )

    # Publishes odom -> base_link. Started after the wheel odometry is flowing so the
    # filter does not have to re-initialise once the controllers come up.
    ekf_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_config, {"use_sim_time": True}],
            )
        ],
        condition=IfCondition(ekf),
    )

    # SLAM: slam_toolbox's node is a LifecycleNode. Started bare (e.g. with `ros2 run`)
    # it stays unconfigured: it subscribes to nothing and logs no error, which looks
    # exactly like a silent hang. So it must be configured and then activated, the way
    # slam_toolbox's own online_async_launch.py does it.
    slam_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            slam_params_file,
            os.path.join(pkg_description, "config", "slam_sim.yaml"),
            {"use_sim_time": True},
        ],
        condition=IfCondition(slam_ready),
    )

    slam_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(slam_ready),
    )

    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(slam_ready),
    )

    return LaunchDescription([
        declare_world_name,
        declare_use_rviz,
        declare_teleop,
        declare_teleop_keyboard,
        declare_bridge_lidar,
        declare_ekf,
        declare_slam,
        declare_slam_params,
        gazebo,
        joint_state_broadcaster_spawner,
        wc_control_spawner,
        rviz,
        teleop_converter,
        keyboard_node,
        lidar_bridge,
        imu_republisher,
        ekf_node,
        slam_node,
        slam_configure,
        slam_activate,
    ])
