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
                            LogInfo, RegisterEventHandler, TimerAction)
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
    bridge_camera = LaunchConfiguration("bridge_camera")
    ekf = LaunchConfiguration("ekf")
    slam = LaunchConfiguration("slam")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav2 = LaunchConfiguration("nav2")
    use_collision_monitor = LaunchConfiguration("use_collision_monitor")
    map_file = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    bt_xml = LaunchConfiguration("bt_xml")

    pkg_navigation = get_package_share_directory("wheelchair_navigation")

    # SLAM needs odom -> base_link (the EKF) plus the bridged /scan.
    slam_ready = PythonExpression([
        "'", slam, "' == 'true' and '", ekf, "' == 'true' and '", nav2, "' != 'true'",
    ])

    # Nav2 localizes with AMCL on a saved map and drives to goals on its own.
    nav2_ready = PythonExpression([
        "'", nav2, "' == 'true' and '", ekf, "' == 'true' and '", bridge_lidar, "' == 'true'",
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
    declare_bridge_camera = DeclareLaunchArgument(
        "bridge_camera", default_value="false",
        description="Bridge the three sim RGB-D cameras to the RealSense topic names "
                    "(/<cam>/color/image_raw, /<cam>/aligned_depth_to_color/image_raw, "
                    "/<cam>/color/camera_info), used by scripts/voice_nav.py"
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
    declare_nav2 = DeclareLaunchArgument(
        "nav2", default_value="false",
        description="Run the full Nav2 stack (AMCL localization + planner + controller + "
                    "behaviour trees) so the wheelchair drives to goals by itself. "
                    "Requires ekf:=true and bridge_lidar:=true, and takes precedence "
                    "over slam:= because both would publish map -> odom."
    )
    declare_map = DeclareLaunchArgument(
        "map", default_value="",
        description="Saved map YAML that Nav2 localizes against, e.g. maps/small_house.yaml. "
                    "Leave empty to build one first with slam:=true and teleop:=true."
    )
    declare_nav2_params_file = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=os.path.join(pkg_navigation, "config", "nav2_params_3cam_v29.yaml"),
        description="The real-robot Nav2 config to reuse, so simulated tuning matches the "
                    "chair; config/nav2_sim.yaml is layered on top for sim time and /scan"
    )
    declare_bt_xml = DeclareLaunchArgument(
        "bt_xml",
        default_value=os.path.join(pkg_navigation, "behavior_tree",
                                   "wheelchair_robust_nav_v3.xml"),
        description="Nav2 behaviour tree, same as the one the real robot uses"
    )
    declare_collision_monitor = DeclareLaunchArgument(
        "use_collision_monitor", default_value="false",
        description="Insert the nav2_collision_monitor stop/slowdown layer between the "
                    "velocity smoother and the motors. Off by default, matching "
                    "wheelchair_fusion_nav.launch.py. When off, the smoothed command "
                    "goes straight to the wheels."
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

    # The three sim RGB-D cameras (urdf/wc_gazebo.xacro) are remapped to the RealSense topic
    # names, so scripts/voice_nav.py runs unchanged on the chair.
    sim_cameras = ["camera", "mapping_camera", "right_camera"]
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        arguments=[a for c in sim_cameras for a in (
            f"/{c}/image@sensor_msgs/msg/Image[gz.msgs.Image",
            f"/{c}/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            f"/{c}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        )],
        remappings=[r for c in sim_cameras for r in (
            (f"/{c}/image", f"/{c}/color/image_raw"),
            (f"/{c}/depth_image", f"/{c}/aligned_depth_to_color/image_raw"),
            (f"/{c}/camera_info", f"/{c}/color/camera_info"),
        )],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(bridge_camera),
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
            # The EKF reads the axle odometry shifted to base_link (see the script).
            Node(
                package="wheelchair_description",
                executable="odom_axle_to_base.py",
                name="odom_axle_to_base",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_config, {
                    "odom0": "/wc_control/odom_base",
                    # Sim only: the wheels' own x, y drift with their heading
                    # (castor slip, 45 deg off after one trip), so fuse only
                    # forward velocity and let the IMU yaw integrate position.
                    "odom0_config": [False, False, False, False, False, False,
                                     True, False, False, False, False, False,
                                     False, False, False],
                    "use_sim_time": True}],
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

    # =========================================================================
    # NAV2 - autonomous navigation on a saved map
    # =========================================================================
    # Command chain, identical to the real robot:
    #   planner/controller -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel
    #   -> collision_monitor -> /cmd_vel_safe -> TwistStamped bridge
    #   -> /wc_control/cmd_vel -> diff_drive_controller -> motors
    # Recovery behaviours are remapped to /cmd_vel_nav too, otherwise BackUp and
    # Spin publish straight to /cmd_vel and skip the velocity smoother.
    nav2_common = {"use_sim_time": True}
    nav2_sim_params = os.path.join(pkg_description, "config", "nav2_sim.yaml")

    controller_server = Node(
        package="nav2_controller", executable="controller_server",
        name="controller_server", output="screen",
        parameters=[nav2_params_file, nav2_sim_params, nav2_common],
        remappings=[("cmd_vel", "cmd_vel_nav")],
        condition=IfCondition(nav2_ready),
    )
    smoother_server = Node(
        package="nav2_smoother", executable="smoother_server",
        name="smoother_server", output="screen",
        parameters=[nav2_params_file, nav2_sim_params, nav2_common],
        condition=IfCondition(nav2_ready),
    )
    planner_server = Node(
        package="nav2_planner", executable="planner_server",
        name="planner_server", output="screen",
        parameters=[nav2_params_file, nav2_sim_params, nav2_common],
        condition=IfCondition(nav2_ready),
    )
    behavior_server = Node(
        package="nav2_behaviors", executable="behavior_server",
        name="behavior_server", output="screen",
        parameters=[nav2_params_file, nav2_sim_params, nav2_common],
        remappings=[("cmd_vel", "cmd_vel_nav")],
        condition=IfCondition(nav2_ready),
    )
    bt_navigator = Node(
        package="nav2_bt_navigator", executable="bt_navigator",
        name="bt_navigator", output="screen",
        parameters=[nav2_params_file, nav2_sim_params, nav2_common,
                    {"default_nav_to_pose_bt_xml": bt_xml,
                     "default_nav_through_poses_bt_xml": bt_xml}],
        condition=IfCondition(nav2_ready),
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother", executable="velocity_smoother",
        name="velocity_smoother", output="screen",
        parameters=[nav2_params_file, nav2_sim_params, nav2_common],
        remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
        condition=IfCondition(nav2_ready),
    )
    waypoint_follower = Node(
        package="nav2_waypoint_follower", executable="waypoint_follower",
        name="waypoint_follower", output="screen",
        parameters=[nav2_params_file, nav2_sim_params, nav2_common],
        remappings=[("cmd_vel", "cmd_vel_nav")],
        condition=IfCondition(nav2_ready),
    )
    # The collision monitor owns the tail of the chain: it reads the smoothed
    # command and publishes the clamped one that reaches the motors. It is off by
    # default, so the wheels are fed from /cmd_vel directly in that case.
    collision_monitor = Node(
        package="nav2_collision_monitor", executable="collision_monitor",
        name="collision_monitor", output="screen",
        parameters=[nav2_params_file, nav2_sim_params, nav2_common],
        remappings=[("cmd_vel_in", "cmd_vel"), ("cmd_vel_out", "cmd_vel_safe")],
        condition=IfCondition(use_collision_monitor),
    )
    map_server = Node(
        package="nav2_map_server", executable="map_server",
        name="map_server", output="screen",
        parameters=[nav2_params_file, nav2_sim_params, nav2_common,
                    {"yaml_filename": map_file}],
        condition=IfCondition(nav2_ready),
    )
    # The TwistStamped bridge is what actually drives the wheels: diff_drive runs
    # inside Gazebo and consumes /wc_control/cmd_vel. It reads /cmd_vel_safe when
    # the collision monitor is enabled and /cmd_vel when it is not.
    nav2_cmd_vel_bridge = Node(
        package="scripts", executable="twist_stamped_teleop",
        name="twist_stamped_teleop", output="screen",
        parameters=[nav2_common],
        remappings=[
            ("cmd_vel_in", PythonExpression([
                "'", use_collision_monitor, "' == 'true' and 'cmd_vel_safe' or 'cmd_vel'"])),
            ("cmd_vel_out", "wc_control/cmd_vel"),
        ],
        condition=IfCondition(nav2_ready),
    )
    # amcl is spawned here rather than as a plain Node because the lifecycle
    # manager has to bring it up with the rest of the stack.
    amcl_node = LifecycleNode(
        package="nav2_amcl", executable="amcl", name="amcl",
        namespace="", output="screen",
        parameters=[nav2_params_file, nav2_sim_params, nav2_common],
        condition=IfCondition(nav2_ready),
    )
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="nav2_lifecycle_manager", output="screen",
        parameters=[nav2_common, {"autostart": True, "node_names": PythonExpression([
            "['map_server', 'amcl', 'controller_server', 'smoother_server', ",
            "'planner_server', 'behavior_server', 'bt_navigator', ",
            "'velocity_smoother', 'waypoint_follower'] + ",
            "(['collision_monitor'] if '", use_collision_monitor, "' == 'true' else [])",
        ])}],
        condition=IfCondition(nav2_ready),
    )

    # The servers need the map and the wheel odometry to exist before they can
    # activate, so the whole stack comes up on a timer rather than at t=0.
    nav2_startup = TimerAction(
        period=18.0,
        actions=[map_server, amcl_node, controller_server, smoother_server,
                 planner_server, behavior_server, bt_navigator,
                 velocity_smoother, waypoint_follower,
                 nav2_cmd_vel_bridge, nav2_lifecycle_manager],
    )

    nav2_ready_message = TimerAction(
        period=28.0,
        actions=[LogInfo(msg=[
            "\n", "=" * 70, "\n",
            "  Nav2 is up - the wheelchair now drives on its own.\n",
            "  Set a goal with the RViz '2D Goal Pose' tool, or:\n",
            "    ros2 action send_goal /navigate_to_pose ",
            "nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: 'map'}, ",
            "pose: {position: {x: 3.0, y: 2.0}, orientation: {w: 1.0}}}\"\n",
            "=" * 70, "\n",
        ])],
        condition=IfCondition(nav2_ready),
    )

    return LaunchDescription([
        declare_world_name,
        declare_use_rviz,
        declare_teleop,
        declare_teleop_keyboard,
        declare_bridge_lidar,
        declare_bridge_camera,
        declare_ekf,
        declare_slam,
        declare_slam_params,
        declare_nav2,
        declare_map,
        declare_nav2_params_file,
        declare_bt_xml,
        declare_collision_monitor,
        gazebo,
        joint_state_broadcaster_spawner,
        wc_control_spawner,
        rviz,
        teleop_converter,
        keyboard_node,
        lidar_bridge,
        camera_bridge,
        imu_republisher,
        ekf_node,
        slam_node,
        slam_configure,
        slam_activate,
        nav2_startup,
        nav2_ready_message,
        collision_monitor,
    ])
