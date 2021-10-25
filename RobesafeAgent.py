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
import visualization_msgs.msg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, NavSatStatus
from t4ac_msgs.msg import CarControl
from cv_bridge import CvBridge, CvBridgeError

# Math and geometry imports

import math
import numpy as np

# Custom functions imports

from modules.geometric_functions import euler_to_quaternion
from modules.bridge_functions import build_camera_info, build_camera_info_from_file, cv2_to_imgmsg, image_rectification, get_input_route_list
from t4ac_global_planner_ros.src.lane_waypoint_planner import LaneWaypointPlanner

### Auxiliar functions

def get_entry_point():
    """
    Return the name of our agent class. This will be used to automatically instantiate our agent.
    """
    return 'RobesafeAgent'

# Class

class RobesafeAgent(AutonomousAgent):
    def setup(self, path_to_conf_file):
        print("\033[1;31m"+"Start init configuration: "+'\033[0;m')

         ### Layers variables

        self.time_sleep = 1

        ## Perception

        # Cameras

        # TODO: Build camera info using the ROI K matrix and build here, NOT IN THE CALLBACK, ADDING THE CAMERA_FRAME
        # AS ARGUMENT

        self.encoding = "bgra8"
        self.bridge = CvBridge()
        self.width = 1080
        self.height = 540
        self.fov = 60
        self.f = self.width / (2.0 * math.tan(self.fov * math.pi / 360.0))
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0
        self.camera_position_3Dcenter = np.array([0, 0]).reshape(-1,2) # With respect to the first camera, that represents the 0,0

        ## Control

        self.Kp = 0.175
        self.Ki = 0.002
        self.error_sum = 0
        self.steer_cmd = 0
        self.speed_cmd = 0

        ## Mapping

        self.trajectory_flag = True

        ### Track

        self.track = Track.MAP

        ## ROS communications

        # Publishers

        self.pub_gnss_pose = rospy.Publisher('/t4ac/localization/gnss_pose', Odometry, queue_size=1)
        self.pub_gnss_fix = rospy.Publisher('/t4ac/localization/fix', NavSatFix, queue_size=1)
        self.pub_raw_image = rospy.Publisher('/t4ac/perception/sensors/raw_image', Image, queue_size=100)
        self.pub_raw_image_info = rospy.Publisher('/t4ac/perception/sensors/raw_image_info', CameraInfo, queue_size = 100)
        self.pub_rectified_image = rospy.Publisher('/t4ac/perception/sensors/rectified_image', Image, queue_size=100)
        self.pub_rectified_image_info = rospy.Publisher('/t4ac/perception/sensors/rectified_image_info', CameraInfo, queue_size = 100)
        self.pub_waypoints_visualizator = rospy.Publisher("/mapping_planning/debug/waypoints", visualization_msgs.msg.Marker, queue_size = 10)
       
        # Subscribers

        self.sub_cmd_vel = rospy.Subscriber('/t4ac/control/cmd_vel', CarControl, self.read_cmd_vel_callback)

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

        # T4AC roslaunch
        
        self.t4ac_architecture_launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/workspace/team_code/catkin_ws/src/t4ac_config_layer/t4ac_utils_ros/launch/t4ac_config.launch"])
        self.t4ac_architecture_launch.start()
        # self.t4ac_config_launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/workspace/team_code/catkin_ws/src/t4ac_config_layer/t4ac_utils_ros/launch/t4ac_config.launch"])
        # self.t4ac_control_launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/workspace/team_code/catkin_ws/src/t4ac_control_layer/launch/t4ac_lqr_ros.launch"])
        # self.t4ac_localization_launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/workspace/team_code/catkin_ws/src/t4ac_localization_layer/launch/t4ac_localization.launch"])
        # self.t4ac_decision_making_launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/workspace/team_code/catkin_ws/src/t4ac_decision_making_layer/launch/t4ac_petrinets_ros.launch"])
        
        # self.t4ac_config_launch.start()
        # self.t4ac_control_launch.start()
        # self.t4ac_localization_launch.start()
        # self.t4ac_decision_making_launch.start()

        rospy.loginfo("started")

        ## Init the node

        rospy.init_node('robesafe_agent', anonymous=True)

        ## Frames

        self.map_frame = rospy.get_param('/t4ac/frames/map')
        self.base_link_frame = rospy.get_param('/t4ac/frames/base_link')
        self.camera_frame = rospy.get_param('/t4ac/frames/camera')

        ## Sensors

        # N.B. 
        # 1. The origin of the sensors in CARLA is located in the geometric center of the vehicle z-filtered 
        #    (that is, ground plane) -> A sensor placed at 0,0,0 position is in the middle and ground plane
        # 2. roll,pitch and yaw from our TF file (e.g. t4ac_config.launch) is different to roll, pitch, yaw in sensors function
        #    roll,pitch and yaw from our TF file represent the rotation of our device w.r.t. the origin of the vehicle in RADIANS
        #    roll,pitch and yaw from sensor function represent the position of the axis, pointing the sensor towards x-direction

        # e.g. You may have a sensor roll=0, pitch=90 and yaw=0, since you start with a LiDAR frame, x would point upwards, y
        # leftwards and z backwards (that is, the ORIENTATION of the sensor). BUT, in the configuration file you specify the final position
        # of your frame w.r.t. the origin (a camera would be z frontwards, x rightwards and y downwards)

        self.camera_position = rospy.get_param('/t4ac/tf/base_link_to_camera_tf')
        self.xcam, self.ycam, self.zcam = self.camera_position[:3]
        self.gnss_position = rospy.get_param('/t4ac/tf/base_link_to_gnss_tf')
        self.xgnss, self.ygnss, self.zgnss = self.gnss_position[:3]
        
        print("\033[1;31m"+"End init configuration: "+'\033[0;m')

    # Specify your sensors
    def sensors(self):
        sensors =  [
                    {'type': 'sensor.camera.rgb', 'x': self.xcam, 'y': self.ycam, 'z': self.zcam, 
                      'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,'width': 
                      self.width, 'height': self.height, 'fov': self.fov, 'id': 'Camera'},
                    {'type': 'sensor.other.gnss', 'x': self.xgnss, 'y': self.ygnss, 'z': self.zgnss, 'id': 'GNSS'},
                    {'type': 'sensor.other.imu', 'x': self.xgnss, 'y': self.ygnss, 'z': self.zgnss, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'id': 'IMU'},
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
        gnss = (input_data['GNSS'][1])
        imu = (input_data['IMU'][1])
        camera = (input_data['Camera'][1])

        if self.trajectory_flag:
            print("Keys: ", input_data.keys())
            hd_map = (input_data['OpenDRIVE'][1])
            # hd_map = hd_map['opendrive'].split('\n')
            distance_among_waypoints = 2
            self.LWP = LaneWaypointPlanner(hd_map['opendrive'],1)
            self.route = self.LWP.calculate_waypoint_route_multiple(distance_among_waypoints, self._global_plan_world_coord, 1)
            self.trajectory_flag = False

        # Publish until control has started   
        if actual_speed < 0.2:
            self.LWP.publish_waypoints(self.route)

        # # Publish markers
        # print("Global plan: ", self._global_plan_world_coord)
        # waypoints_markers = get_input_route_list(hd_map, self._global_plan_world_coord)
        # self.pub_waypoints_visualizator.publish(waypoints_markers)
        
        # Callbacks
        self.gnss_imu_callback(gnss, imu, current_ros_time)
        self.cameras_callback(camera, current_ros_time)
        control = self.control_callback(actual_speed)

        return control 

    # Callbacks

    def cameras_callback(self, raw_image, current_ros_time):
        """
        Return the information of the correspondin camera as a sensor_msgs.Image ROS message based on a string 
        that contains the camera information
        """

        raw_image = cv2_to_imgmsg(raw_image, self.encoding)
        raw_image.header.stamp = current_ros_time
        raw_image.header.frame_id = self.camera_frame
        # raw_image_info = build_camera_info(self.width, self.height, self.f, 
        #                                    self.camera_position_3Dcenter[0,0], self.camera_position_3Dcenter[0,1], 
        #                                    current_ros_time)
        
        self.pub_raw_image.publish(raw_image)
        # self.pub_raw_image_info.publish(raw_image_info)

        # Rectify the image

        cv_image = self.bridge.imgmsg_to_cv2(raw_image, desired_encoding='passthrough')
        cwd = os.getcwd()
        camera_parameters_path = '/workspace/team_code/modules/camera_parameters/'
        roi_rectified_image = image_rectification(cv_image, camera_parameters_path)

        roi_rectified_image = cv2_to_imgmsg(roi_rectified_image, self.encoding)
        roi_rectified_image.header.stamp = current_ros_time
        roi_rectified_image.header.frame_id = self.camera_frame 
        # rectified_image_info = build_camera_info_from_file(self.camera_frame, camera_parameters_path, 
        #                                                    self.camera_position_3Dcenter[0], current_ros_time)
        rectified_image_info = build_camera_info(roi_rectified_image.width, roi_rectified_image.height, self.f, 
                                                 self.camera_position_3Dcenter[0,0], self.camera_position_3Dcenter[0,1], 
                                                 current_ros_time)
        self.pub_rectified_image.publish(roi_rectified_image)
        self.pub_rectified_image_info.publish(rectified_image_info)

    def gnss_imu_callback(self, gnss, imu, current_ros_time):
        """
        Return UTM position (x,y,z) and orientation of the ego-vehicle as a nav_msgs.Odometry ROS message based on the
        gnss information (WGS84) and imu (to compute the orientation)
            GNSS    ->  latitude =  gnss[0] ; longitude = gnss[1] ; altitude = gnss[2]
            IMU     ->  accelerometer.x = imu[0] ; accelerometer.y = imu[1] ; accelerometer.z = imu[2] ; 
                        gyroscope.x = imu[3]  ;  gyroscope.y = imu[4]  ;  gyroscope.z = imu[5]  ;  compas = imu[6]
        """
        
        # Read and publish GNSS data
        gnss_msg = NavSatFix()
        gnss_msg.header.stamp = current_ros_time
        gnss_msg.header.frame_id = 'gnss'
        gnss_msg.latitude = gnss[0]
        gnss_msg.longitude = gnss[1]
        gnss_msg.altitude = gnss[2]
        gnss_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
        gnss_msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO
        self.pub_gnss_fix.publish(gnss_msg)

        gnss_pose_msg = Odometry()
        gnss_pose_msg.header.frame_id = self.map_frame
        gnss_pose_msg.child_frame_id = self.base_link_frame
        gnss_pose_msg.header.stamp = current_ros_time

        # Convert Geographic (latitude, longitude) to UTM (x,y) coordinates
        gnss_msg.latitude = -gnss_msg.latitude
        EARTH_RADIUS_EQUA = 6378137.0   # pylint: disable=invalid-name
        scale = math.cos(gnss_msg.latitude * math.pi / 180.0)
        x = scale * gnss_msg.longitude * math.pi * EARTH_RADIUS_EQUA / 180.0 
        # Negative y to correspond to carla documentations
        y = - scale * EARTH_RADIUS_EQUA * math.log(math.tan((90.0 + gnss_msg.latitude) * math.pi / 360.0))  
        #################################################################################################
        #####  It doesn't work with CARLA, they use an approximation to perform the conversion #######
        # import utm
        # ....
        # gnss_msg.latitude = -gnss_msg.latitude # Since in CARLA Towns the y-reference is the opposite
        # u = utm.from_latlon(gnss_msg.latitude, gnss_msg.longitude)
        # x = u[0] - self.origin[0]
        # y = u[1] - self.origin[1]
        #################################################################################################

        # Read IMU data -> Yaw angle is used to give orientation to the gnss pose 
        roll = 0
        pitch = 0
        compass = imu[6]

        if (0 < compass < math.radians(180)):
            yaw = -compass + math.radians(90)
        else:
            yaw = -compass + math.radians(450)
                
        [qx, qy, qz, qw] = euler_to_quaternion(roll, pitch, yaw)

        gnss_pose_msg.pose.pose.position.x = x
        gnss_pose_msg.pose.pose.position.y = y 
        gnss_pose_msg.pose.pose.position.z = 0

        gnss_pose_msg.pose.pose.orientation.x = qx
        gnss_pose_msg.pose.pose.orientation.y = qy
        gnss_pose_msg.pose.pose.orientation.z = qz
        gnss_pose_msg.pose.pose.orientation.w = qw

        gnns_error = 1.5
        gnss_pose_msg.pose.covariance = np.diag([gnns_error, gnns_error, gnns_error, 0, 0, 0]).ravel()

        self.pub_gnss_pose.publish(gnss_pose_msg)
 
    def control_callback(self, actual_speed):
        """
        Return the current state of the vehicle regarding the control layer
        """
        error_speed = self.speed_cmd - actual_speed  # distance away from setpoint [m/s]
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
        control.steer = - self.steer_cmd
        control.throttle = throttle
        control.brake = brake
        control.hand_brake = False

        # print("Speed: ",self.speed_cmd, "  Steer: ", self.steer_cmd)

        return control

    def read_cmd_vel_callback(self, cmd_vel):
        """
        Return the current cmd_vel command
        """
        self.speed_cmd = cmd_vel.velocity
        self.steer_cmd = cmd_vel.steer

    # Destroy the agent

    def destroy(self):
        
        """
        Destroy (clean-up) the agent
        :return:
        """

        self.t4ac_architecture_launch.shutdown()
        # self.t4ac_config_launch.shutdown()
        # self.t4ac_control_launch.shutdown()
        # self.t4ac_localization_launch.shutdown()
        # self.t4ac_decision_making_launch.shutdown()

        pass