#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the ScenarioManager implementations.
It must not be modified and is for reference only!
"""

from __future__ import print_function
import signal
import sys
import time

import numpy as np
import t4ac_msgs.msg
import tf
import math

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog

from leaderboard.autoagents.agent_wrapper import AgentWrapper, AgentError
from leaderboard.envs.sensor_interface import SensorReceivedNoData
from leaderboard.utils.result_writer import ResultOutputProvider

def apply_tf(source_location, transform):  
    """
    Input: t4ac_msgs.msg.Node() in the source frame
    Output: t4ac_msgs.msg.Node() in the target frame 
    """
    centroid = np.array([0.0,0.0,0.0,1.0]).reshape(4,1)

    try:
        centroid[0,0] = source_location.x 
        centroid[1,0] = source_location.y
        centroid[2,0] = source_location.z
    except:
        centroid[0,0] = source_location[0] # LiDAR points (3,)
        centroid[1,0] = source_location[1]
        centroid[2,0] = source_location[2]

    aux = np.dot(transform,centroid) 

    target_location = t4ac_msgs.msg.Node()
    target_location.x = aux[0,0]
    target_location.y = aux[1,0]
    target_location.z = aux[2,0]

    return target_location

class ScenarioManager(object):

    """
    Basic scenario manager class. This class holds all functionality
    required to start, run and stop a scenario.

    The user must not modify this class.

    To use the ScenarioManager:
    1. Create an object via manager = ScenarioManager()
    2. Load a scenario via manager.load_scenario()
    3. Trigger the execution of the scenario manager.run_scenario()
       This function is designed to explicitly control start and end of
       the scenario execution
    4. If needed, cleanup with manager.stop_scenario()
    """


    def __init__(self, timeout, debug_mode=False):
        """
        Setups up the parameters, which will be filled at load_scenario()
        """
        self.scenario = None
        self.scenario_tree = None
        self.scenario_class = None
        self.ego_vehicles = None

        self.ego_yaw_origin = None
        self.ego_yaw_diff = 0.0
        self.ego_pitch_origin = None
        self.ego_pitch_diff = 0.0
        self.ego_roll_origin = None
        self.ego_roll_diff = 0.0

        self.other_actors = None

        self._debug_mode = debug_mode
        self._agent = None
        self._running = False
        self._timestamp_last_run = 0.0
        self._timeout = float(timeout)

        # Used to detect if the simulation is down
        watchdog_timeout = max(5, self._timeout - 2)
        self._watchdog = Watchdog(watchdog_timeout)

        # Avoid the agent from freezing the simulation
        agent_timeout = watchdog_timeout - 1
        self._agent_watchdog = Watchdog(agent_timeout)

        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        self.end_game_time = None

        # Register the scenario tick as callback for the CARLA world
        # Use the callback_id inside the signal handler to allow external interrupts
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signum, frame):
        """
        Terminate scenario ticking when receiving a signal interrupt
        """
        self._running = False

    def cleanup(self):
        """
        Reset all parameters
        """
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        self.end_game_time = None

    def load_scenario(self, scenario, agent, rep_number):
        """
        Load a new scenario
        """

        GameTime.restart()
        self._agent = AgentWrapper(agent)
        self.scenario_class = scenario
        self.scenario = scenario.scenario
        self.scenario_tree = self.scenario.scenario_tree
        self.ego_vehicles = scenario.ego_vehicles
        self.other_actors = scenario.other_actors
        self.repetition_number = rep_number

        # To print the scenario tree uncomment the next line
        # py_trees.display.render_dot_tree(self.scenario_tree)

        self._agent.setup_sensors(self.ego_vehicles[0], self._debug_mode)

    def run_scenario(self):
        """
        Trigger the start of the scenario and wait for it to finish/fail
        """
        self.start_system_time = time.time()
        self.start_game_time = GameTime.get_time()

        self._watchdog.start()
        self._running = True

        while self._running:
            timestamp = None
            world = CarlaDataProvider.get_world()
            if world:
                snapshot = world.get_snapshot()
                if snapshot:
                    timestamp = snapshot.timestamp
            if timestamp:
                self._tick_scenario(timestamp)

    def _tick_scenario(self, timestamp):
        """
        Run next tick of scenario and the agent and tick the world.
        """

        if self._timestamp_last_run < timestamp.elapsed_seconds and self._running:
            self._timestamp_last_run = timestamp.elapsed_seconds

            self._watchdog.update()
            # Update game time and actor information
            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()

            try:
                ego_action = self._agent()

            # Special exception inside the agent that isn't caused by the agent
            except SensorReceivedNoData as e:
                raise RuntimeError(e)

            except Exception as e:
                raise AgentError(e)

            self.ego_vehicles[0].apply_control(ego_action)

            # Tick scenario
            self.scenario_tree.tick_once()

            if self._debug_mode:
                print("\n")
                py_trees.display.print_ascii_tree(
                    self.scenario_tree, show_status=True)
                sys.stdout.flush()

            if self.scenario_tree.status != py_trees.common.Status.RUNNING:
                self._running = False

            spectator = CarlaDataProvider.get_world().get_spectator()
            ego_trans = self.ego_vehicles[0].get_transform()

            ego_roll, ego_pitch, ego_yaw = ego_trans.rotation.roll, ego_trans.rotation.pitch, ego_trans.rotation.yaw

            if not self.ego_yaw_origin:
                self.ego_yaw_origin = ego_yaw
                self.ego_pitch_origin = ego_pitch
                self.ego_roll_origin = ego_roll
            else:
                self.ego_yaw_diff = ego_yaw - self.ego_yaw_origin
                self.ego_pitch_diff = ego_pitch - self.ego_pitch_origin
                self.ego_roll_diff = ego_roll - self.ego_roll_origin

            quaternion = tf.transformations.quaternion_from_euler(math.radians(ego_roll), 
                                                                  math.radians(ego_pitch), 
                                                                  math.radians(ego_yaw))

            rot_matrix = tf.transformations.quaternion_matrix(quaternion)
            translation = np.array([0,0,0])
            
            tf_ego = rot_matrix
            tf_ego[:3,3] = tf_ego[:3,3] + translation

            ori_view = t4ac_msgs.msg.Node(-5,3,3.5) # Change this to get a different perspective
            new_view = apply_tf(ori_view, tf_ego)

            # 3D view

            spectator.set_transform(carla.Transform(ego_trans.location + carla.Location(x=new_view.x,y=new_view.y,z=new_view.z), 
                                                           #carla.Rotation(roll=0,pitch=-25,yaw=20+self.ego_yaw_origin+self.ego_yaw_diff)))
                                                           carla.Rotation(roll=0-self.ego_roll_origin-self.ego_roll_diff,
                                                                          pitch=-25-self.ego_pitch_origin-self.ego_pitch_diff,
                                                                          yaw=-20+self.ego_yaw_origin+self.ego_yaw_diff)))

        if self._running and self.get_running_status():
            CarlaDataProvider.get_world().tick(self._timeout)

    def get_running_status(self):
        """
        returns:
           bool: False if watchdog exception occured, True otherwise
        """
        return self._watchdog.get_status()

    def stop_scenario(self):
        """
        This function triggers a proper termination of a scenario
        """
        self._watchdog.stop()

        self.end_system_time = time.time()
        self.end_game_time = GameTime.get_time()

        self.scenario_duration_system = self.end_system_time - self.start_system_time
        self.scenario_duration_game = self.end_game_time - self.start_game_time

        if self.get_running_status():
            if self.scenario is not None:
                self.scenario.terminate()

            if self._agent is not None:
                self._agent.cleanup()
                self._agent = None

            self.analyze_scenario()

    def analyze_scenario(self):
        """
        Analyzes and prints the results of the route
        """
        global_result = '\033[92m'+'SUCCESS'+'\033[0m'

        for criterion in self.scenario.get_criteria():
            if criterion.test_status != "SUCCESS":
                global_result = '\033[91m'+'FAILURE'+'\033[0m'

        if self.scenario.timeout_node.timeout:
            global_result = '\033[91m'+'FAILURE'+'\033[0m'

        ResultOutputProvider(self, global_result)
