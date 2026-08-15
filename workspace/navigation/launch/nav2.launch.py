# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    params_dir = get_package_share_directory('navigation')
    params_dir_file = os.path.join(params_dir, 'param', 'nav2.yaml')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
   
    # nav2.launch.py의 49번 줄 주변
    # 시뮬레이터와 EKF가 글로벌 /tf에 퍼블리시하므로, Nav2도 글로벌 /tf를 듣도록 리매핑을 제거합니다.
    remappings = [
                    ('/tf', '/tf'),
                    ('/tf_static', '/tf_static'),
                    # 🌟 Nav2 가 속도를 내보낼 토픽. 기본은 로봇의 `cmd_vel` 이지만
                    # 인자로 바꿀 수 있다.
                    #
                    # 드론이 이걸 쓴다. 드론은 Nav2 와 드라이버 사이에
                    # local_altitude_avoider 를 끼워서 `linear.z`(고도)만 따로 채운다.
                    # Nav2 는 `linear.z` 를 항상 0 으로 두므로 z 축이 통째로 비어 있고,
                    # 그래서 수평(Nav2)과 수직(회피기)이 서로 싸우지 않는다.
                    ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
                     ('/odom', ['/', namespace, '/odom']),
                 ]
    # remappings = []


    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'robot_base_frame': [namespace, '/base_link'], # 🌟 base_link를 ugv1/base_link로 변경
        'global_frame': [namespace, '/map'],
        'local_frame': [namespace, '/odom'], # 🌟 behavior_server(리커버리)용 odom 프레임
        'odom_topic':  [namespace, '/odom'],
        # 🌟 static layer 가 구독할 맵. 기본은 `/{ns}/map` 이지만 인자로 바꿀 수 있다.
        #
        # 드론이 이걸 쓴다. 드론의 `/{ns}/map` 은 **여러 고도의 합집합**이라
        # 자기 플래너에 주면 지금 고도에서는 뚫려 있는 곳을 못 지나간다.
        # 그래서 드론은 `map_topic:=/{ns}/map_active`(현재 순항 고도 한 층)를 넘긴다.
        # 자세한 근거는 webots_python/drone_layer_mapper.py 모듈 주석.
        'map_topic': LaunchConfiguration('map_topic'),
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local
    }

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=params_dir_file,
            description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'),

        DeclareLaunchArgument(
            'map_subscribe_transient_local', default_value='false',
            description='Whether to set the map subscriber QoS to transient local'),

        DeclareLaunchArgument(
            'map_topic', default_value=['/', namespace, '/map'],
            description='static layer 가 구독할 맵 토픽. 드론은 /{ns}/map_active 를 넘긴다'),

        DeclareLaunchArgument(
            'cmd_vel_topic', default_value=['/', namespace, '/cmd_vel'],
            description='Nav2 가 속도를 내보낼 토픽. 드론은 /{ns}/cmd_vel_nav 를 넘겨 '
                        'local_altitude_avoider 를 중간에 끼운다'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            namespace=namespace,
            output='screen',
            parameters=[configured_params,
                        {'local_costmap.local_costmap.ros__parameters.global_frame': [namespace, '/odom']}
                        ],
            remappings=remappings),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=namespace,
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=namespace,
            output='screen',
            # 🌟 local_costmap 과 같은 이유의 예외 처리다.
            #
            # behavior_server(spin/backup)가 쓰는 프레임 파라미터의 이름은 Humble 에서
            # `global_frame` 이고, 기본값이 네임스페이스 없는 "odom" 이다. 위쪽
            # param_substitutions 의 `global_frame` 은 `{ns}/map` 이라 여기엔 맞지 않고,
            # 애초에 nav2.yaml 의 behavior_server 블록에 그 키가 없으면 RewrittenYaml 이
            # 바꿀 대상 자체가 없어서 기본값 "odom" 이 그대로 남는다.
            #
            # 그러면 리커버리가 `odom` 프레임을 찾다가 매번 즉시 실패한다.
            #   [transformPoseInTargetFrame] target frame "odom" does not exist
            #   [behavior_server] Initial checks failed for spin/backup -> Aborting
            # 주행 자체는 되는데 리커버리가 전부 죽으므로, BT 가 리커버리를 한 번이라도
            # 타는 순간(경로 재계산 지연 등) 목표가 통째로 ABORT 된다. 실측으로 확인.
            #
            # 그래서 노드 레벨에서 직접 덮어쓴다 (노드 파라미터가 yaml 보다 우선).
            parameters=[configured_params,
                        {'global_frame': [namespace, '/odom']}
                        ],
            remappings=remappings),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=namespace,
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            namespace=namespace,
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),

    ])
