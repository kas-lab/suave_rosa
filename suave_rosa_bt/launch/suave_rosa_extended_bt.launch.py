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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mission_type = LaunchConfiguration('mission_type')
    result_filename = LaunchConfiguration('result_filename')

    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='time_constrained_mission',
        description='Desired mission type' +
                    '[time_constrained_mission or const_dist_mission]'
    )

    result_filename_arg = DeclareLaunchArgument(
        'result_filename',
        default_value='',
        description='Name of the results file'
    )

    pkg_suave_rosa = get_package_share_directory(
        'suave_rosa')
    suave_rosa_path = os.path.join(
        pkg_suave_rosa,
        'launch',
        'suave_rosa_base.launch.py')

    mission_config = os.path.join(
        get_package_share_directory('suave_missions'),
        'config',
        'mission_config.yaml'
    )

    suave_rosa_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_rosa_path),
        launch_arguments={
            'mission_type': mission_type,
            'result_filename': result_filename,
            'mission_config': mission_config,
            'db_name': 'suave_rosa_extended_bt',
        }.items()
    )

    suave_rosa_bt_node = Node(
        package='suave_rosa_bt',
        executable='suave_rosa_extended_bt',
        parameters=[mission_config]
    )

    return LaunchDescription([
        mission_type_arg,
        result_filename_arg,
        suave_rosa_base,
        suave_rosa_bt_node,
    ])