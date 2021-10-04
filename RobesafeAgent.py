# CARLA imports
import carla
from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track

# General use
import os
import time
import sys

import rospy
import rosnode
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from std_msgs.msg import Float64, Float32, Header

# Math and geometry imports

import shapely.geometry as geom
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate

# Auxiliar functions

def get_entry_point():
    """
    Return the name of our agent class. This will be used to automatically instantiate our agent.
    """
    return 'RobesafeAgent'

# Class

class RobesafeAgent(AutonomousAgent):
    def setup(self, path_to_conf_file):

        ### Track

        self.track = Track.MAP

        ### ROS communications

        ## Publishers

        ## Subscribers

        self.sub_throttle = rospy.Subscriber('/control/throttle', Float64, self.read_throttle_callback)
        self.control_node_name = 'control_node'

        ### Launch the architecture


        try:
            ros_nodes = rosnode.get_node_names()
            print("ROS nodes: ", ros_nodes)
            control_node_name = '/'+rospy.get_param("/t4ac/control/node_name")
            print("Control node name: ", control_node_name)
            if control_node_name not in ros_nodes: 
                os.system('roslaunch control_test control_test.launch &') # Control test
        except:
            print("Evaluating first scenario, roscore not launched yet") 
            os.system('roslaunch control_test control_test.launch &') # Control test
        
        ### Init the node
        rospy.init_node("robesafe_agent", anonymous=True)

        # time_sleep = 1
        # time.sleep(time_sleep)

    # Specify your sensors

    def sensors(self):

        sensors =  [{'type': 'sensor.speedometer',  'reading_frequency': 20, 'id': 'Speed'}
                    ]            
        return sensors

    # Run step function

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation. Gather the sensor information
        """
        current_ros_time = rospy.Time.now()

        # Get sensor data

        actual_speed = (input_data['Speed'][1])['speed']
        
        # Callbacks

        control = self.control_callback(actual_speed)

        return control

    # Auxiliar class functions
    def control_callback(self, actual_speed):
        """
        Return the current state of the vehicle regarding the control layer
        """

        # Return control

        control = carla.VehicleControl()
        control.steer = 0
        control.throttle = self.throttle_cmd
        control.brake = 0
        control.hand_brake = False

        return control

    def read_throttle_callback(self, throttle):
        """
        Return the state of the steering wheel of the ego-vehicle
        """
        self.throttle_cmd = throttle.data

    # Destroy the agent

    def destroy(self):
        """
        Destroy (clean-up) the agent
        :return:
        """
        pass