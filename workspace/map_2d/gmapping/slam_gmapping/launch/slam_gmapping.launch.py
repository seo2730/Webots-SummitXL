# 수정 예시: slam_gmapping/launch/slam_gmapping.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. namespace LaunchConfiguration 선언
    namespace = LaunchConfiguration('namespace')

    ekf_launch_dir = os.path.join(get_package_share_directory('kalman_filter_localization'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ekf_launch_dir, 'ekf.launch.py')),
            launch_arguments={'namespace': namespace}.items()
        ),
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            namespace=namespace, # 2. Node에 namespace 전달 (필수)
            output='screen',
            parameters=[{'use_sim_time': True}],
            # 3. 리매핑 시 반드시 앞쪽에 '/'가 없는 상대 경로('scan', 'map')를 사용해야 합니다.
            remappings=[
                ('/scan', 'scan'),   # 노드 내부에서 절대경로를 쓴다면 상대경로로 강제 리매핑
                ('/map', 'map')
            ]
        )
    ])
