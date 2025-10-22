import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
  IncludeLaunchDescription,
  DeclareLaunchArgument,
  GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
  LaunchConfiguration,
  PathJoinSubstitution,
  PythonExpression,
)
from launch.conditions import IfCondition      temp_arg,

from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
  package_name = "igvc25"

  use_sim_time = LaunchConfiguration('use_sim_time')



  # https://docs.nav2.org/index.html

  # MARK: Lcyl Mngr
  # https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_lifecycle_manager

  nav_params = os.path.join(
      get_package_share_directory(package_name),
      'config',
      'navigation.yaml'
  )

  lifecycle_nodes = [
      'controller_server',
      'smoother_server',
      'planner_server',
      'behavior_server',
      'velocity_smoother',
      'collision_monitor',
      'bt_navigator',
      # 'waypoint_follower',
      # 'docking_server',
  ];

  lsm = Node(
      package='nav2_lifecycle_manager',
      executable='lifecycle_manager',
      name='navigation_manager',
      parameters=[
          {'node_names': lifecycle_nodes},
          {'autostart': True},
          {'use_sim_time': use_sim_time}
      ]
  );

  # MARK: BT Nvgt
  # https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_bt_navigator
  # https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html
  # responsible for determining what behaviors to do, and actually move robot
  bt_navigator = Node(
      package='nav2_bt_navigator',
      executable='bt_navigator',
      parameters=[
          nav_params,
          {'use_sim_time': use_sim_time}
      ],
  );

  # MARK: Ctrl Svr
  # https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_controller
  # https://docs.nav2.org/configuration/packages/configuring-controller-server.html
  # responsible for publishing command velocities for the robot
  # generates path for robot to follow, and checks on robot progress
  controller_server = Node(
      package='nav2_controller',
      executable='controller_server',
      parameters=[
          nav_params,
          {'use_sim_time': use_sim_time}
      ],
      remappings=[
          ('cmd_vel', 'cmd_vel_nav')
      ]
  );

  # MARK: Smtr Svr
  # https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_smoother
  # https://docs.nav2.org/configuration/packages/configuring-smoother-server.html
  # responsible for smoothing planned path from planner server for controller server
  # Need to set up bt-navigator to get this to work
  smoother_server = Node(
      package='nav2_smoother',
      executable='smoother_server',
      parameters=[
          nav_params,
          {'use_sim_time': use_sim_time}
      ],
  );

  # MARK: Plnr Svr
  # https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_planner
  # https://docs.nav2.org/configuration/packages/configuring-planner-server.html
  # responsible for generating a feasible path for the robot to go on
  planner_server = Node(
      package='nav2_planner',
      executable='planner_server',
      parameters=[
          nav_params,
          {'use_sim_time': use_sim_time}
      ],
  );

  # MARK: Bhvr Svr
  # https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_behaviors
  # https://docs.nav2.org/configuration/packages/configuring-behavior-server.html
  # responsivle for handling robot behaiviors specifies by bt-navigator
  behavior_server = Node(
      package='nav2_behaviors',
      executable='behavior_server',
      parameters=[
          nav_params,
          {'use_sim_time': use_sim_time}
      ],
  );

  # MARK: Clsn Mntr
  # https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_collision_monitor
  # https://docs.nav2.org/configuration/packages/configuring-collision-monitor.html
  # resposible for slowing and stoping robot before it hits something
  collision_monitor = Node(
      package='nav2_collision_monitor',
      executable='collision_monitor',
      parameters=[
          nav_params,
          {'use_sim_time': use_sim_time}
      ],
  );

  # MARK: Vel Smtr
  # https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_smoother
  # needs setup in bt w/ bt=navigator to do anything at all
  vel_smoother = Node(
      package='nav2_velocity_smoother',
      executable='velocity_smoother',
      parameters=[
          nav_params,
          {'use_sim_time': use_sim_time}
      ],
      remappings=[
          ('cmd_vel', 'cmd_vel_nav')
      ]
  );

  # MARK: Wpnt Flwr
  # https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_waypoint_follower
  # https://docs.nav2.org/configuration/packages/configuring-waypoint-follower.html

  # tutorial: https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html
  # responsible for
  waypoint_follower = Node(
      package='nav2_waypoint_follower',
      executable='waypoint_follower',
      parameters=[
          nav_params,
          {'use_sim_time': use_sim_time}
      ],
  )

  # MARK: Launch!
  return LaunchDescription(
    [
      DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'
      ),

      # Nav2 communication servers
      controller_server,
      smoother_server,
      planner_server,
      behavior_server,

      # Nav2 addtional bits
      collision_monitor,
      vel_smoother,
      # waypoint_follower,

      # Nav2 navigator & bond
      bt_navigator,
      lsm,
    ]
  )
