
#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

"""
Created on Tue Oct 5 14:20:29 2021
@author: RobeSafe research group
"""

# CARLA imports
import carla
from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track

# General use
import os
import time
import sys

# ROS imports

import rospy
import rosnode
import roslaunch
import rosgraph
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.point_cloud2 import create_cloud
from std_msgs.msg import Float64, Float32, Header

# Math and geometry imports

import shapely.geometry as geom
import math
import numpy as np
import matplotlib.pyplot as plt
import utm
from scipy import interpolate

# Custom functions imports

from modules.geometric_functions import euler_to_quaternion, unit_vector
from modules.bridge_functions import get_input_route_list

sys.path.insert(0,'/workspace/team_code/catkin_ws/src/t4ac_mapping_planning/t4ac_map_builder/src')
from builder_classes import T4ac_Location
from path_planner import PathPlanner

# Auxiliar functions

def get_entry_point():
    """
    Return the name of our agent class. This will be used to automatically instantiate our agent.
    """
    return 'RobesafeAgent'

# Class

class RobesafeAgent(AutonomousAgent):
    def setup(self, path_to_conf_file):
        print("\033[1;31m"+"Start init configuration: "+'\033[0;m')

        ## Layers variables

        self.time_sleep = 1

        # Control

        self.Kp = 0.175
        self.Ki = 0.002
        self.error_sum = 0
        self.steer_cmd = 0
        self.speed_cmd = 0

        # Mapping-Planning

        self.origin = utm.from_latlon(0, 0) # lat_origin, lon_origin
        self.offset_compass = -5/2*math.pi
        self.trajectory_flag = False
        self.flag_path_planner = False
        self.hd_map = []
        self.waypoints = []

        ### Track

        self.track = Track.MAP

        ## ROS communications

        # Publishers

        self.pub_waypoints_path = rospy.Publisher('/mapping_planning/waypoints', Path, queue_size=1000)
        self.pub_gps_data = rospy.Publisher('/localization/gps_pose', Odometry, queue_size=10)

        # Subscribers

        self.sub_steer = rospy.Subscriber('/control/steer', Float64, self.read_steer_callback)
        self.sub_speed = rospy.Subscriber('/control/speed', Float64, self.read_speed_callback)

        ## Launch the architecture

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        try:
            # rosgraph.Master('/rosout').getPid()
            print("\033[1;31m"+"Try roscore"+'\033[0;m')
            rosgraph.Master('/rostopic').getPid()
        except:
            print("\033[1;31m"+"Except roscore"+'\033[0;m')
            launch = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_files=[], is_core=True)
            launch.start()

        self.localization_launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/workspace/team_code/catkin_ws/src/sec_localization/launch/sec_localization.launch"])
        self.control_launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/workspace/team_code/catkin_ws/src/t4ac_control/t4ac_controller/launch/controller.launch"])
        self.localization_launch.start()
        self.control_launch.start()

        rospy.loginfo("started")

        ## Init the node

        rospy.init_node('robesafe_agent', anonymous=False)

        print("\033[1;31m"+"End init configuration: "+'\033[0;m')

    # Specify your sensors

    def sensors(self):
        sensors =  [
                    {'type': 'sensor.other.gnss', 'x': -1.425, 'y': 0.0, 'z': 1.60, 'id': 'GPS'},
                    {'type': 'sensor.other.imu', 'x': -1.425, 'y': 0.0, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'id': 'IMU'},
                    {'type': 'sensor.opendrive_map', 'reading_frequency': 1, 'id': 'OpenDRIVE'},
                    {'type': 'sensor.speedometer',  'reading_frequency': 20, 'id': 'Speed'},
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
        gps = (input_data['GPS'][1])
        imu = (input_data['IMU'][1])

        if not self.trajectory_flag:
            print("Keys: ", input_data.keys())
            self.hd_map = (input_data['OpenDRIVE'][1])
            self.hd_map = self.hd_map['opendrive'].split('\n')
            self.trajectory_flag = True
            self.input_route_list = get_input_route_list(self.origin, self._global_plan)

        # Callbacks

        self.trajectory_callback(current_ros_time)
        self.gps_callback(gps, imu, current_ros_time)
        control = self.control_callback(actual_speed)

        return control 

    # Callbacks

    def trajectory_callback(self, current_ros_time):
        """
        This callback returns the calculated waypoints as nav_msgs.Path ROS message using our path_planner, spaced by a distance 
        "distance_among_waypoints", based on a T4ac_location points list
        """
        while not rospy.is_shutdown():
            waypoints_path = Path()
            waypoints_path.header.frame_id = "/map"
            waypoints_path.header.stamp = current_ros_time

            distance_among_waypoints = 5

            if not self.flag_path_planner:
                path_planner = PathPlanner(self.hd_map)
                self.waypoints = path_planner.get_route(self.input_route_list,distance_among_waypoints)
                self.flag_path_planner = True

            #x = []
            #y = []

            if self.waypoints is not None:
                for wp in self.waypoints:
                    pose = PoseStamped()
                    pose.pose.position.x = wp.transform.location.x
                    pose.pose.position.y = -wp.transform.location.y
                    pose.pose.position.z = 0
                    waypoints_path.poses.append(pose)

                    #x.append(wp.transform.location.x)
                    #y.append(wp.transform.location.y)
            """
            # Plot the calculated waypoints
            plt.plot(x,y,'k--',label='Control polygon',marker='o',markerfacecolor='red')
            plt.legend(loc='best')
            plt.axis([min(x)-1, max(x)+1, min(y)-1, max(y)+1])
            plt.title('Path planner waypoints')
            plt.show()
            """
            self.pub_waypoints_path.publish(waypoints_path)
            
            return

    def gps_callback(self, gps, imu, current_ros_time):
        """
        Return UTM position (x,y,z) and orientation of the ego-vehicle as a nav_msgs.Odometry ROS message based on the
        gps information (WGS84) and imu (to compute the orientation)
        """
        while not rospy.is_shutdown():
            gps_data = Odometry()
            gps_data.header.frame_id = "map"
            gps_data.child_frame_id = "ros_center"
            gps_data.header.stamp = current_ros_time
            
            lat = abs(gps[0]) 
            lon = abs(gps[1])
            
            u = utm.from_latlon(lat, lon)
            x = u[0] - self.origin[0]
            y = u[1] - self.origin[1]  

            if (gps[0] > 0):
                y = -y
            if (gps[1] < 0):
                x=-x

            roll = 0
            pitch = 0
            yaw = imu[6] + self.offset_compass

            if (yaw < -math.pi):
                yaw = 2*math.pi + yaw

            [qx, qy, qz, qw] = euler_to_quaternion(roll, pitch, yaw)

            gps_data.pose.pose.position.x = x
            gps_data.pose.pose.position.y = y
            gps_data.pose.pose.position.z = 0

            gps_data.pose.pose.orientation.x = qx
            gps_data.pose.pose.orientation.y = qy
            gps_data.pose.pose.orientation.z = qz
            gps_data.pose.pose.orientation.w = qw

            error_gps = 1.5
            gps_data.pose.covariance = np.diag([error_gps, error_gps, error_gps, 0, 0, 0]).ravel()

            self.pub_gps_data.publish(gps_data)

            return
            
    def control_callback(self, actual_speed):
        """
        Return the current state of the vehicle regarding the control layer
        """
        error_speed = self.speed_cmd - actual_speed  # distance away from setpoint
        self.error_sum += (error_speed*self.Ki)

        if (self.error_sum > 0.5):
            self.error_sum = 0.5
        if (self.error_sum < -0.5):
            self.error_sum = -0.5

        # Throttle

        throttle = (error_speed*self.Kp) + self.error_sum

        if (self.speed_cmd == 0):
            self.error_sum = 0  # Reset PI

        # Brake

        brake = 0
        if (throttle < 0):
            brake = 1 #-throttle
            throttle = 0
        if (throttle > 1):
            throttle = 1

        # Return control

        control = carla.VehicleControl()
        control.steer = self.steer_cmd
        control.throttle = throttle
        control.brake = brake
        control.hand_brake = False

        # print("Speed: ",self.speed_cmd, "  Steer: ", self.steer_cmd)

        return control

    def read_steer_callback(self, steer):
        """
        Return the state of the steering wheel of the ego-vehicle
        """
        self.steer_cmd = steer.data

    def read_speed_callback(self, speed):
        """
        Return the current speed of the ego-vehicle
        """
        self.speed_cmd = speed.data

    # Destroy the agent

    def destroy(self):
        
        """
        Destroy (clean-up) the agent
        :return:
        """

        self.localization_launch.shutdown()
        self.control_launch.shutdown()

        pass