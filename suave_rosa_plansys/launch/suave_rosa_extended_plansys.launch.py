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
        default_value='time_constrained_mission',
        description='Desired mission type' +
                    '[time_constrained_mission or const_dist_mission]'
    )

    result_path = LaunchConfiguration('result_path')
    result_path_arg = DeclareLaunchArgument(
        'result_path',
        default_value='~/suave/results',
        description='Path where to save the results'
    )
    result_filename_arg = DeclareLaunchArgument(
        'result_filename',
        default_value='',
        description='Name of the results file'
    )

    pkg_suave_rosa = get_package_share_directory(
        'suave_rosa')
    suave_rosa_base_launch_path = os.path.join(
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

    data_path_ = "[{0}, {1}]".format(
        os.path.join(pkg_suave_rosa, 'config', 'suave.tql'),
        os.path.join(pkg_suave_rosa, 'config', 'suave_extended.tql'),
    )
    suave_rosa_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_rosa_base_launch_path),
        launch_arguments={
            'mission_type': mission_type,
            'result_path': result_path,
            'result_filename': result_filename,
            'mission_config': mission_config,
            'db_name': 'suave_rosa_extended_plansys',
            'data_path': data_path_
        }.items()
    )

    suave_rosa_plansys_path = get_package_share_directory('suave_rosa_plansys')
    plansys_path = get_package_share_directory('plansys2_bringup')
    plansys2_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                plansys_path,
                'launch',
                'plansys2_bringup_launch_distributed.py')),
            launch_arguments={
                'model_file':
                    suave_rosa_plansys_path + '/pddl/suave_domain.pddl',
                'problem_file':
                    suave_rosa_plansys_path + '/pddl/suave_extended_problem.pddl',
            }.items()
        )

    rosa_plansys_controller_node = Node(
        package='suave_rosa_plansys',
        executable='suave_rosa_controller',
        # parameters=[mission_config]
        parameters=[{'rosa_actions': [
            'search_pipeline', 'inspect_pipeline', 'recharge']}]
    )

    start_robot_pddl_action_node = Node(
        package='suave_rosa_plansys',
        executable='action_start_robot',
        # parameters=[mission_config]
        parameters=[{'action_name': 'start_robot'}]
    )

    search_pipeline_pddl_action_node = Node(
        package='suave_rosa_plansys',
        executable='action_search_pipeline',
        # parameters=[mission_config]
        parameters=[{'action_name': 'search_pipeline'}]
    )

    inspect_pipeline_pddl_action_node = Node(
        package='suave_rosa_plansys',
        executable='action_inspect_pipeline',
        # parameters=[mission_config]
        parameters=[{'action_name': 'inspect_pipeline'}]
    )

    recharge_pddl_action_node = Node(
        package='suave_rosa_plansys',
        executable='action_recharge',
        # parameters=[mission_config]
        parameters=[{'action_name': 'recharge'}]
    )

    return LaunchDescription([
        mission_type_arg,
        result_path_arg,
        result_filename_arg,
        silent_arg,
        OpaqueFunction(function=configure_logging),
        mission_config_arg,
        suave_rosa_base,
        plansys2_bringup,
        rosa_plansys_controller_node,
        start_robot_pddl_action_node,
        search_pipeline_pddl_action_node,
        inspect_pipeline_pddl_action_node,
        recharge_pddl_action_node,
    ])
