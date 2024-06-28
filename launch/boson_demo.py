# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from launch import LaunchDescription
import launch
import launch.events
import launch_ros.event_handlers
import launch_ros.events.lifecycle
import launch_ros.actions

import lifecycle_msgs.msg

def generate_launch_description():

    boson_node = launch_ros.actions.LifecycleNode(
                        name='flir_boson_usb', namespace='',
                        package='flir_boson_usb', executable='BosonLifeCycle',
                        output='screen',
                        parameters=[{
                            'dev': "/dev/video2",
                            'sensor_type': 'boson640'
                        }])

    configure_boson = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(boson_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
        )
    )

    activate_boson = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(boson_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
        )
    )

    return LaunchDescription([
        boson_node,
        configure_boson,
        launch.actions.TimerAction(period=1.0, actions=[activate_boson])
    ])
