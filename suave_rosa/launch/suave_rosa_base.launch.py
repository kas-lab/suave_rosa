# Copyright 2023 Gustavo Rezende Silva
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
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    mission_type = LaunchConfiguration('mission_type')
    result_filename = LaunchConfiguration('result_filename')
    mission_config = LaunchConfiguration('mission_config')
    db_name = LaunchConfiguration('db_name')
    data_path = LaunchConfiguration('data_path')

    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='time_constrained_mission',
        description='Mission name for logging'
    )

    result_path = LaunchConfiguration('result_path')
    result_path_arg = DeclareLaunchArgument(
        'result_path',
        default_value='~/suave/results',
        description='Path where to save the results'
    )

    result_filename_arg = DeclareLaunchArgument(
        'result_filename',
        default_value='rosa_results',
        description='Name of the results file'
    )

    mission_config_default = os.path.join(
        get_package_share_directory('suave_missions'),
        'config',
        'mission_config.yaml'
    )

    mission_config_arg = DeclareLaunchArgument(
        'mission_config',
        default_value=mission_config_default,
        description='Mission configuration file'
    )

    db_name_arg = DeclareLaunchArgument(
        'db_name',
        default_value='suave_rosa',
        description='ROSA db name'
    )

    pkg_suave_rosa = get_package_share_directory(
        'suave_rosa')
    data_path_ = "[{}]".format(
        os.path.join(pkg_suave_rosa, 'config', 'suave.tql'))
    data_path_arg = DeclareLaunchArgument(
        'data_path',
        default_value=data_path_,
        description='typeQL file to use for data'
    )

    pkg_rosa_bringup = get_package_share_directory(
        'rosa_bringup')
    rosa_bringup_launch_path = os.path.join(
        pkg_rosa_bringup,
        'launch',
        'rosa_bringup.launch.py')

    pkg_suave_path = get_package_share_directory(
        'suave')
    suave_launch_path = os.path.join(
        pkg_suave_path,
        'launch',
        'suave.launch.py')

    rosa_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rosa_bringup_launch_path),
        launch_arguments={
            'data_path': data_path,
            'database_name': db_name,
            'force_data': 'True',
            'force_database': 'True',
            'infer': 'True',
        }.items()
    )

    suave_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_launch_path),
        launch_arguments={
            'task_bridge': 'False', 'system_modes': 'False'}.items()
    )

    mission_metrics_node = Node(
        package='suave_metrics',
        executable='mission_metrics',
        name='mission_metrics',
        parameters=[mission_config, {
            'adaptation_manager': 'rosa',
            'mission_name': mission_type,
            'result_path': result_path,
            'result_filename': result_filename,
        }],
    )

    typedb_server = ExecuteProcess(
        cmd=['typedb', 'server'],
        name='typedb_server',
        output='screen'
    )

    # Include another launch file after typedb starts
    event_launch_rosa_bringup = RegisterEventHandler(
        OnProcessStart(
            target_action=typedb_server,
            on_start=[rosa_bringup]
        )
    )

    return LaunchDescription([
        mission_type_arg,
        result_path_arg,
        result_filename_arg,
        mission_config_arg,
        db_name_arg,
        data_path_arg,
        suave_launch,
        typedb_server,
        event_launch_rosa_bringup,
        mission_metrics_node,
    ])
