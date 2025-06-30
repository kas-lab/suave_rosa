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
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    silent = LaunchConfiguration('silent')
    silent_arg = DeclareLaunchArgument(
        'silent',
        default_value='false',
        description='Suppress all output (launch logs + node logs)'
    )
    def configure_logging(context, *args, **kwargs):
        if silent.perform(context) == 'true':
            import logging
            logging.getLogger().setLevel(logging.CRITICAL)
        return []
    
    mission_type = LaunchConfiguration('mission_type')
    result_filename = LaunchConfiguration('result_filename')

    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='suave',
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

    pkg_suave_rosa = get_package_share_directory(
        'suave_rosa')
    suave_rosa_path = os.path.join(
        pkg_suave_rosa,
        'launch',
        'suave_rosa_base.launch.py')

    mission_config_default = os.path.join(
        get_package_share_directory('suave_missions'),
        'config',
        'mission_config.yaml'
    )

    mission_config = LaunchConfiguration('mission_config')
    mission_config_arg = DeclareLaunchArgument(
        'mission_config',
        default_value=mission_config_default,
        description='Mission config full path'
    )

    suave_rosa_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_rosa_path),
        launch_arguments={
            'mission_type': mission_type,
            'result_path': result_path,
            'result_filename': result_filename,
            'mission_config': mission_config,
            'db_name': 'suave_rosa_bt',
        }.items()
    )

    suave_rosa_bt_node = Node(
        package='suave_rosa_bt',
        executable='suave_rosa_bt',
        parameters=[mission_config]
    )

    return LaunchDescription([
        silent_arg,
        OpaqueFunction(function=configure_logging),
        mission_type_arg,
        mission_config_arg,
        result_filename_arg,
        result_path_arg,
        suave_rosa_base,
        suave_rosa_bt_node,
    ])
