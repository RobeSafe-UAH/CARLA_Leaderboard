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
import cv2

# ROS imports

import rospy
import rosnode
import roslaunch
import rosgraph
import visualization_msgs.msg
from nav_msgs.msg import Odometry
from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, NavSatStatus, PointCloud2, PointField
from t4ac_msgs.msg import CarControl
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64, Float32, Header

# Math and geometry imports

import math
import numpy as np

# Custom functions imports

from modules.geometric_functions import euler_to_quaternion
from modules.bridge_functions import build_camera_info, build_camera_info_from_file, cv2_to_imgmsg, image_rectification, get_input_route_list, lidar_string_to_array
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

        self.calibrate_camera = False
        self.encoding = "bgra8"
        self.bridge = CvBridge()
        self.width = 1920 # 2060, 1920
        self.height = 1080 # 1080, 540
        self.fov = 80 # 80, 60
        self.camera_parameters_path = '/workspace/team_code/modules/camera_parameters/'
        self.config = 'camera_parameters_' + str(self.width) + '_' + str(self.height) + '_' + str(self.fov) + '/' 
        self.f = self.width / (2.0 * math.tan(self.fov * math.pi / 360.0))
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0
        self.camera_position_3Dcenter = np.array([0, 0]).reshape(-1,2) # With respect to the first camera, that represents the 0,0

        # Camera center parameters
        self.width_center = 1920 # 2060, 1920
        self.height_center = 1080 # 1080, 540
        self.fov_center = 80 # 80, 60
        self.f_center = self.width_center / (2.0 * math.tan(self.fov_center * math.pi / 360.0))
        self.cx_center = self.width_center / 2.0
        self.cy_center = self.height_center / 2.0
        self.config_center = 'camera_parameters_' + str(self.width_center) + '_' + str(self.height_center) + '_' + str(self.fov_center) + '/' 

        # Camera right parameters
        self.width_right = 1920 # 2060, 1920
        self.height_right = 1080 # 1080, 540
        self.fov_right = 80 # 80, 60
        self.f_right = self.width_right / (2.0 * math.tan(self.fov_right * math.pi / 360.0))
        self.cx_right = self.width_right / 2.0
        self.cy_right = self.height_right / 2.0
        self.config_right = 'camera_parameters_' + str(self.width_right) + '_' + str(self.height_right) + '_' + str(self.fov_right) + '/' 
       
        # Camera right parameters
        self.width_left = 1920 # 2060, 1920
        self.height_left = 1080 # 1080, 540
        self.fov_left = 80 # 80, 60
        self.f_left = self.width_left / (2.0 * math.tan(self.fov_left * math.pi / 360.0))
        self.cx_left = self.width_left / 2.0
        self.cy_left = self.height_left / 2.0
        self.config_left = 'camera_parameters_' + str(self.width_left) + '_' + str(self.height_left) + '_' + str(self.fov_right) + '/' 

        self.cameras_id = ["camera_center", "camera_left", "camera_right"]

        # LiDAR

        self.half_cloud = []
        self.lidar_count = 0

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

        self.pub_image_raw_center = rospy.Publisher('/t4ac/perception/sensors/center/image_raw', Image, queue_size=100)
        self.pub_image_raw_left = rospy.Publisher('/t4ac/perception/sensors/left/image_raw', Image, queue_size=100)
        self.pub_image_raw_right = rospy.Publisher('/t4ac/perception/sensors/right/image_raw', Image, queue_size=100)
        # self.pub_image_raw_rear  = rospy.Publisher('/t4ac/perception/sensors/rear/image_raw', Image, queue_size=100)

        self.pub_camera_info_center = rospy.Publisher('/t4ac/perception/sensors/center/camera_info', CameraInfo, queue_size = 100)
        self.pub_camera_info_left = rospy.Publisher('/t4ac/perception/sensors/left/camera_info', CameraInfo, queue_size=100)
        self.pub_camera_info_right = rospy.Publisher('/t4ac/perception/sensors/right/camera_info', CameraInfo, queue_size=100)
        # self.pub_camera_info_rear  = rospy.Publisher('/t4ac/perception/sensors/rear/camera_info', CameraInfo, queue_size=100)

        self.pub_image_rect_center = rospy.Publisher('/t4ac/perception/sensors/center/image_rect', Image, queue_size=100)
        self.pub_image_rect_left = rospy.Publisher('/t4ac/perception/sensors/left/image_rect', Image, queue_size=100)
        self.pub_image_rect_right = rospy.Publisher('/t4ac/perception/sensors/right/image_rect', Image, queue_size=100)
        # self.pub_image_rect_rear = rospy.Publisher('/t4ac/perception/sensors/rear/image_rect', Image, queue_size=100)

        self.pub_camera_info_rect_center = rospy.Publisher('/t4ac/perception/sensors/center/rect/camera_info', CameraInfo, queue_size = 100)
        self.pub_camera_info_rect_left = rospy.Publisher('/t4ac/perception/sensors/left/rect/camera_info', CameraInfo, queue_size = 100)
        self.pub_camera_info_rect_right = rospy.Publisher('/t4ac/perception/sensors/right/rect/camera_info', CameraInfo, queue_size = 100)
        # self.pub_camera_info_rect_rear = rospy.Publisher('/t4ac/perception/sensors/rear/rect/camera_info', CameraInfo, queue_size = 100)

        self.pub_lidar_pointcloud = rospy.Publisher('/t4ac/perception/sensors/lidar', PointCloud2, queue_size=10)
       
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

        rospy.loginfo("Techs4AgeCar architecture started")

        ## Init the node

        rospy.init_node('robesafe_agent', anonymous=True)

        ## Frames

        self.map_frame = rospy.get_param('/t4ac/frames/map')
        self.base_link_frame = rospy.get_param('/t4ac/frames/base_link')

        camera_center_frame = rospy.get_param('/t4ac/frames/camera_center')
        camera_left_frame = rospy.get_param('/t4ac/frames/camera_left')
        camera_right_frame = rospy.get_param('/t4ac/frames/camera_right')
        # camera_rear_frame = rospy.get_param('/t4ac/frames/camera_rear')
        self.cameras_frames = [camera_center_frame, camera_left_frame, camera_right_frame]
        
        self.lidar_frame = rospy.get_param('/t4ac/frames/lidar')

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

        self.camera_position = rospy.get_param('/t4ac/tf/base_link_to_camera_center_tf')
        self.xcam_center, self.ycam_center, self.zcam_center = self.camera_position[:3]

        self.camera_position = rospy.get_param('/t4ac/tf/base_link_to_camera_left_tf')
        self.xcam_left, self.ycam_left, self.zcam_left = self.camera_position[:3]
        self.yaw_left = (self.camera_position[5] + 1.57079632679) * (-90 / 1.5707963)

        self.camera_position = rospy.get_param('/t4ac/tf/base_link_to_camera_right_tf')
        self.xcam_right, self.ycam_right, self.zcam_right = self.camera_position[:3]
        self.yaw_right = (self.camera_position[5] + 1.57079632679) * (-90 / 1.57079632679)

        self.lidar_position = rospy.get_param('/t4ac/tf/base_link_to_lidar_tf')
        self.xlidar, self.ylidar, self.zlidar = self.lidar_position[:3]
        self.gnss_position = rospy.get_param('/t4ac/tf/base_link_to_gnss_tf')
        self.xgnss, self.ygnss, self.zgnss = self.gnss_position[:3]
        
        print("\033[1;31m"+"End init configuration: "+'\033[0;m')

    # Specify your sensors

    def sensors(self):
        sensors =  [
                    {'type': 'sensor.camera.rgb', 'x': self.xcam_center, 'y': self.ycam_center, 'z': self.zcam_center, 
                      'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,'width': 
                      self.width_center, 'height': self.height_center, 'fov': self.fov_center, 'id': 'Camera_center'},

                    # {'type': 'sensor.camera.rgb', 'x': self.xcam_left, 'y': self.ycam_left, 'z': self.zcam_left, 
                    #   'roll': 0.0, 'pitch': 0.0, 'yaw': self.yaw_left,'width': 
                    #   self.width_left, 'height': self.height_left, 'fov': self.fov_left, 'id': 'Camera_left'},

                    # {'type': 'sensor.camera.rgb', 'x': self.xcam_right, 'y': self.ycam_right, 'z': self.zcam_right, 
                    #   'roll': 0.0, 'pitch': 0.0, 'yaw': self.yaw_right,'width': 
                    #   self.width_right, 'height': self.height_right, 'fov': self.fov_right, 'id': 'Camera_right'},

                    # {'type': 'sensor.camera.rgb', 'x': self.xcam_rear, 'y': self.ycam_rear, 'z': self.zcam_rear, 
                    #   'roll': 0.0, 'pitch': 0.0, 'yaw': 180.0,'width': 
                    #   self.width, 'height': self.height, 'fov': self.fov, 'id': 'Camera_rear'},  

                    {'type': 'sensor.lidar.ray_cast', 'x': self.xlidar, 'y': self.ylidar, 'z': self.zlidar, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'id': 'LiDAR'},
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

        cameras = []
        for key in input_data.keys():
            if "Camera" in key:
               cameras.append(input_data[key][1]) 
 
        lidar = (input_data['LiDAR'][1])

        if self.trajectory_flag:
            hd_map = (input_data['OpenDRIVE'][1])
            distance_among_waypoints = 2
            rospy.set_param('/t4ac/map_parameters/distance_among_waypoints', distance_among_waypoints)
            self.LWP = LaneWaypointPlanner(hd_map['opendrive'],1)
            self.map_name = self.LWP.map_name
            rospy.set_param('/t4ac/map_parameters/map_name', self.map_name)
            self.route = self.LWP.calculate_waypoint_route_multiple(distance_among_waypoints, self._global_plan_world_coord, 1)
            self.trajectory_flag = False

        # Publish until control has started   

        if actual_speed < 0.2:
            self.LWP.publish_waypoints(self.route)
        
        # Callbacks

        self.gnss_imu_callback(gnss, imu, current_ros_time)
        self.cameras_callback(cameras, current_ros_time)

        self.lidar_callback(lidar, current_ros_time) 
        control = self.control_callback(actual_speed)

        return control 

    # Callbacks

    def lidar_callback(self, lidar, current_ros_time):
        """
        Return the LiDAR pointcloud as a sensor_msgs.PointCloud2 ROS message based on a string that contains
        the LiDAR information
        """
        # while not rospy.is_shutdown():
        self.lidar_count += 1
        if (self.lidar_count % 2 == 0):
            self.lidar_count = 0

            header = Header()
            header.stamp = current_ros_time
            header.frame_id = self.lidar_frame

            whole_cloud = True

            lidar_data = lidar_string_to_array(lidar,self.half_cloud,whole_cloud)

            fields = [
                        PointField('x', 0, PointField.FLOAT32, 1),
                        PointField('y', 4, PointField.FLOAT32, 1),
                        PointField('z', 8, PointField.FLOAT32, 1),
                        PointField('intensity', 12, PointField.FLOAT32, 1),
                        ]
            point_cloud_msg = create_cloud(header, fields, lidar_data)
            self.pub_lidar_pointcloud.publish(point_cloud_msg)
        else:
            self.half_cloud = lidar_string_to_array(lidar)
        # return

    def cameras_callback(self, cameras, current_ros_time):
        """
        Return the information of the correspondin camera as a sensor_msgs.Image ROS message based on a string 
        that contains the camera information
        """

        for i, raw_image in enumerate(cameras):
            raw_image = cv2_to_imgmsg(raw_image, self.encoding)
            raw_image.header.stamp = current_ros_time
            raw_image.header.frame_id = self.cameras_frames[i]
        
            # Rectify the image

            cv_image = self.bridge.imgmsg_to_cv2(raw_image, desired_encoding='passthrough')

            if self.calibrate_camera:
                cv2.imwrite("/workspace/team_code/modules/camera_parameters/distorted_image_" + str(self.cameras_id[i]) + ".png", cv_image)

            roi_rectified_image = image_rectification(cv_image, self.camera_parameters_path+self.config)

            roi_rectified_image = cv2_to_imgmsg(roi_rectified_image, self.encoding)
            roi_rectified_image.header.stamp = current_ros_time
            roi_rectified_image.header.frame_id = self.cameras_frames[i]
            # rectified_image_info = build_camera_info_from_file(self.camera_frame, self.camera_parameters_path+self.config, 
            #                                                    self.camera_position_3Dcenter[0,0], self.camera_position_3Dcenter[0,1], current_ros_time)
            rectified_image_info = build_camera_info(roi_rectified_image.width, roi_rectified_image.height, self.f, 
                                                    self.camera_position_3Dcenter[0,0], self.camera_position_3Dcenter[0,1], 
                                                    current_ros_time, self.cameras_frames[i])


            #  Publish information 
            if (self.cameras_frames[i] == 'ego_vehicle/camera/rgb/center'):
                self.pub_image_raw_center.publish(raw_image)
                # self.pub_camera_info_center.publish(raw_image_info)
                self.pub_image_rect_center.publish(roi_rectified_image)
                self.pub_camera_info_rect_center.publish(rectified_image_info)
            elif (self.cameras_frames[i] == 'ego_vehicle/camera/rgb/left'):
                self.pub_image_raw_left.publish(raw_image)
                # self.pub_camera_info_left.publish(raw_image_info)
                self.pub_image_rect_left.publish(roi_rectified_image)
                self.pub_camera_info_rect_left.publish(rectified_image_info)
            elif (self.cameras_frames[i] == 'ego_vehicle/camera/rgb/right'):
                self.pub_image_raw_right.publish(raw_image)
                # self.pub_camera_info_right.publish(raw_image_info)
                self.pub_image_rect_right.publish(roi_rectified_image)
                self.pub_camera_info_rect_right.publish(rectified_image_info)
            # elif (self.cameras_frames[i] == 'ego_vehicle/camera/rgb/rear'):
            #     self.pub_image_raw_rear.publish(raw_image)
            #     # self.pub_camera_info_rear.publish(raw_image_info)
            #     self.pub_image_rect_rear.publish(roi_rectified_image)
            #     self.pub_camera_info_rect_rear.publish(rectified_image_info)

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

        pass