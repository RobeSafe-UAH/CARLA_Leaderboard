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
import roslaunch
import rosgraph
from nav_msgs.msg import Odometry
from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, NavSatStatus, PointCloud2, PointField
from t4ac_msgs.msg import CarControl, Node
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64, Float32, Header

# Math and geometry imports

import math
import numpy as np

# Custom functions imports

from modules.geometric_functions import euler_to_quaternion
from modules.bridge_functions import build_camera_info, build_camera_info_from_file, cv2_to_imgmsg, image_rectification, get_input_route_list, lidar_string_to_array
from t4ac_global_planner_ros.src.lane_waypoint_planner import LaneWaypointPlanner
from map_parser import signal_parser
from map_parser.test import test_signal_parser
from t4ac_map_monitor_ros.src.modules import markers_module, monitor_classes
import visualization_msgs.msg

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

        self.calibrate_camera = False
        self.encoding = "bgra8"
        self.bridge = CvBridge()
        self.camera_parameters_path = '/workspace/team_code/modules/camera_parameters/'

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
        self.pub_lidar_pointcloud = rospy.Publisher('/t4ac/perception/sensors/lidar', PointCloud2, queue_size=10)

        self.pub_current_traffic_light = rospy.Publisher('/t4ac/mapping/current_traffic_light', Odometry, queue_size=2)
        self.signals_visualizator_pub = rospy.Publisher('/t4ac/mapping/debug/signals', visualization_msgs.msg.Marker, queue_size = 10)
       
        # Subscribers

        self.sub_cmd_vel = rospy.Subscriber('/t4ac/control/cmd_vel', CarControl, self.read_cmd_vel_callback)
        self.sub_pose_ekf = rospy.Subscriber('/t4ac/localization/pose', Odometry, self.read_pose_callback)

        ## Launch the architecture

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        try:
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

        self.lidar_position = rospy.get_param('/t4ac/tf/base_link_to_lidar_tf')
        self.xlidar, self.ylidar, self.zlidar = self.lidar_position[:3]
        self.gnss_position = rospy.get_param('/t4ac/tf/base_link_to_gnss_tf')
        self.xgnss, self.ygnss, self.zgnss = self.gnss_position[:3]
        
        # camera_id = center, right, left, rear
        cameras_id = ["center"]#, "left", "right"]
        self.cameras_parameters = []
        for index, camera_id in enumerate(cameras_id):
            """
            camera_id = center, right, left, rear
            """
            width = 1920 # 2060, 1920
            height = 1080 # 1080, 540
            fov = 80 # 80, 60
            fx = width / (2.0 * math.tan(fov * math.pi / 360.0))
            fy = fx 
            camera_position = rospy.get_param('/t4ac/tf/base_link_to_camera_' + str(camera_id) + '_tf')
            camera_dict = dict({
                                'id': camera_id,
                                'width': width,
                                'height': height,
                                'fov': fov,
                                'fx': fx,
                                'fy': fy,
                                'cx': width / 2,
                                'cy': height / 2,
                                'config_file': 'camera_parameters_' + str(width) + '_' + str(height) + '_' + str(fov) + '/',
                                'image_raw_pub': rospy.Publisher('/t4ac/perception/sensors/' + camera_id + '/image_raw', Image, queue_size=100),
                                'camera_info_pub': rospy.Publisher('/t4ac/perception/sensors/' + camera_id + '/camera_info', CameraInfo, queue_size=100),
                                'image_rect_pub': rospy.Publisher('/t4ac/perception/sensors/' + camera_id + '/image_rect', Image, queue_size=100),
                                'camera_info_rect_pub': rospy.Publisher('/t4ac/perception/sensors/' + camera_id + '/rect/camera_info', CameraInfo, queue_size = 100),
                                'frame': rospy.get_param('/t4ac/frames/camera_' + str(camera_id)),
                                'camera_position_3D': np.array([0, 0]).reshape(-1,2), # With respect to a common frame, that would represent the 0,0.
                                                                                      # If set to 0,0, each camera is an independent frame and after
                                                                                      # obtaining the 3D information they will have to transform to a
                                                                                      # common frame (e.g. all cameras -> front_bumper_center -> map)
                                'x': camera_position[0],
                                'y': camera_position[1],
                                'z': camera_position[2],
                                'yaw': (camera_position[5] + 1.57079632679) * (-90 / 1.57079632679)
            })
            self.cameras_parameters.append(camera_dict)

         
        self.traffic_signals = []
        self.pose_ekf = Odometry()
        self.ego_vehicle_yaw = 0
        self.previous_dist_current_traffic_light = 50000
        self.cont = 0

        print("\033[1;31m"+"End init configuration: "+'\033[0;m')

    # Specify your sensors

    def sensors(self):
        sensors = []
        for index, camera in enumerate(self.cameras_parameters):
            sensors.append({'type': 'sensor.camera.rgb', 'x': camera['x'], 'y': camera['y'], 'z': camera['z'], 
                        'roll': 0.0, 'pitch': 0.0, 'yaw': camera['yaw'], 'width': camera['width'], 
                        'height': camera['height'], 'fov': camera['fov'], 'id': camera['id']}) 
                        
        sensors.append({'type': 'sensor.lidar.ray_cast', 'x': self.xlidar, 'y': self.ylidar, 'z': self.zlidar, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'id': 'LiDAR'})
        sensors.append({'type': 'sensor.other.gnss', 'x': self.xgnss, 'y': self.ygnss, 'z': self.zgnss, 'id': 'GNSS'})
        sensors.append({'type': 'sensor.other.imu', 'x': self.xgnss, 'y': self.ygnss, 'z': self.zgnss, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'id': 'IMU'})
        sensors.append({'type': 'sensor.opendrive_map', 'reading_frequency': 1, 'id': 'OpenDRIVE'})
        sensors.append({'type': 'sensor.speedometer',  'reading_frequency': 20, 'id': 'Speed'})
                    
        return sensors

    # Run step function

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation. Gather the sensor information
        """
        current_ros_time = rospy.Time.now()

        # Get sensor data

        if self.trajectory_flag:
            hd_map = (input_data['OpenDRIVE'][1])
            distance_among_waypoints = 2
            rospy.set_param('/t4ac/map_parameters/distance_among_waypoints', distance_among_waypoints)
            self.LWP = LaneWaypointPlanner(hd_map['opendrive'],1)
            self.map_name = self.LWP.map_name
            rospy.set_param('/t4ac/map_parameters/map_name', self.map_name)
            self.route = self.LWP.calculate_waypoint_route_multiple(distance_among_waypoints, self._global_plan_world_coord, 1)
            self.trajectory_flag = False
            self.traffic_signals = signal_parser.parse_signals(hd_map['opendrive'], 1)
            nodes = []
            for signal in self.traffic_signals:
                # if signal['id'] == 369 or signal['id'] == 370 or signal['id'] == 371:
                node = monitor_classes.Node3D()
                node.x = signal['x']
                node.y = -signal['y']
                node.z = signal['z']
                nodes.append(node)
            signals_marker = markers_module.get_nodes(
                nodes = nodes, rgb = [1,0,1], name = "2", marker_type = 8, 
                scale = 1.5, extra_z = 1, lifetime = 0)
            self.signals_visualizator_pub.publish(signals_marker)

        actual_speed = (input_data['Speed'][1])['speed']
        gnss = (input_data['GNSS'][1])
        imu = (input_data['IMU'][1])

        cameras = []
        for camera in self.cameras_parameters:
            cameras.append(input_data[camera['id']][1]) 
 
        lidar = (input_data['LiDAR'][1])

        # Publish until control has started   

        if actual_speed < 0.2:
            self.LWP.publish_waypoints(self.route)
        
        # Callbacks

        self.gnss_imu_callback(gnss, imu, current_ros_time)
        self.cameras_callback(cameras, current_ros_time)
        self.lidar_callback(lidar, current_ros_time)
        self.traffic_lights_callback(current_ros_time) 
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

    def cameras_callback(self, cameras, current_ros_time):
        """
        Return the information of the correspondin camera as a sensor_msgs.Image ROS message based on a string 
        that contains the camera information
        """
        for index_camera, raw_image in enumerate(cameras):
            raw_image = cv2_to_imgmsg(raw_image, self.encoding)
            raw_image.header.stamp = current_ros_time
            raw_image.header.frame_id = self.cameras_parameters[index_camera]['frame']
            raw_image_K = build_camera_info(raw_image.width, raw_image.height, 
                                               self.cameras_parameters[index_camera]['fx'],
                                               self.cameras_parameters[index_camera]['fy'],
                                               self.cameras_parameters[index_camera]['camera_position_3D'][0,0],
                                               self.cameras_parameters[index_camera]['camera_position_3D'][0,1], 
                                               current_ros_time, self.cameras_parameters[index_camera]['frame'],
                                               distorted_image=True)
        
            # Rectify the image

            cv_image = self.bridge.imgmsg_to_cv2(raw_image, desired_encoding='passthrough')

            if self.calibrate_camera:
                cv2.imwrite("/workspace/team_code/modules/camera_parameters/distorted_image_" + str(self.cameras_parameters[index_camera]['id']) + ".png", cv_image)

            camera_parameters_path = self.camera_parameters_path + self.cameras_parameters[index_camera]['config_file']
            roi_rectified_image = image_rectification(cv_image, raw_image_K, camera_parameters_path)
            
            roi_rectified_image = cv2_to_imgmsg(roi_rectified_image, self.encoding)
            roi_rectified_image.header.stamp = current_ros_time
            roi_rectified_image.header.frame_id = self.cameras_parameters[index_camera]['frame']
            
            fov = self.cameras_parameters[index_camera]['fov']
            roi_fx = roi_rectified_image.width / (2.0 * math.tan(fov * math.pi / 360.0))
            roi_fy = roi_fx
            
            rectified_image_info = build_camera_info_from_file(self.cameras_parameters[index_camera]['frame'],
                                                               self.cameras_parameters[index_camera]['camera_position_3D'][0,0],
                                                               self.cameras_parameters[index_camera]['camera_position_3D'][0,1],
                                                               current_ros_time,
                                                               camera_parameters_path)
            
            # rectified_image_info = build_camera_info(roi_rectified_image.width, roi_rectified_image.height, 
            #                                          roi_fx, roi_fy,
            #                                          self.cameras_parameters[index_camera]['camera_position_3D'][0,0],
            #                                          self.cameras_parameters[index_camera]['camera_position_3D'][0,1], 
            #                                          current_ros_time, self.cameras_parameters[index_camera]['frame'])

            self.cameras_parameters[index_camera]['image_raw_pub'].publish(raw_image)
            # self.cameras_parameters[index_camera]['camera_info_pub'](raw_image_info)
            self.cameras_parameters[index_camera]['camera_info_rect_pub'].publish(rectified_image_info)
            self.cameras_parameters[index_camera]['image_rect_pub'].publish(roi_rectified_image)

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
        self.ego_vehicle_yaw = yaw

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

        return control

    def read_cmd_vel_callback(self, cmd_vel):
        """
        Return the current cmd_vel command
        """
        self.speed_cmd = cmd_vel.velocity
        self.steer_cmd = cmd_vel.steer

    def read_pose_callback(self, data):
        self.pose_ekf = data
        
    def traffic_lights_callback(self, current_ros_time):
        nearest_signals = []
        distance_th = 40
        consecutive_steps = 12

        vehicle_pose = np.array([self.pose_ekf.pose.pose.position.x, self.pose_ekf.pose.pose.position.y])
        current_traffic_light = Odometry()
        current_traffic_light.header.stamp = current_ros_time
        current_traffic_light.header.frame_id = self.map_frame
        current_traffic_light.pose.pose.position.x = 50000
        current_traffic_light.pose.pose.position.y = 50000 
        current_traffic_light.pose.pose.position.z = 50000 

        for i, signal in enumerate(self.traffic_signals):
            
            signal_pose = np.array([signal['x'], signal['y']])
            dist = np.linalg.norm(vehicle_pose - signal_pose)
        
            if (dist < distance_th):
                nearest_signals.append(signal)

                if (round(signal['yaw'], 4) >= round(2 * math.pi, 4)):
                    signal['yaw'] -= 2 * math.pi
                elif (round(signal['yaw'], 4) <= round(- 2 * math.pi, 4)):
                    signal['yaw'] += 2 * math.pi

                diff_angle = abs(self.ego_vehicle_yaw - signal['yaw'])
                # print('diff_angle: ', diff_angle)
                
                    
                if (diff_angle < 0.52): # 0.52 radians = 30 º, to consider curves
                    if (dist > self.previous_dist_current_traffic_light):
                        self.cont += 1
                        if self.cont > consecutive_steps:
                            self.cont = consecutive_steps
                    else:
                        self.cont -= 1
                        if self.cont < 0:
                            self.cont = 0
                    self.previous_dist_current_traffic_light = dist
                    if (self.cont < consecutive_steps):
                        # print('self.cont: ', self.cont)
                        # print("Entro semaforo: ", signal['id'])                  
                        current_traffic_light.pose.pose.position.x = signal['x']
                        current_traffic_light.pose.pose.position.y = signal['y']
                        current_traffic_light.pose.pose.position.z = signal['z']

                        node = monitor_classes.Node3D()
                        node.x = current_traffic_light.pose.pose.position.x
                        node.y = -current_traffic_light.pose.pose.position.y # -y since RVIZ representation is right-handed rule
                        node.z = current_traffic_light.pose.pose.position.z
                        nodes = []
                        nodes.append(node)
                        nodes.append(node)
                        signals_marker = markers_module.get_nodes(
                        nodes = nodes, rgb = [0,1,0], name = "current_traffic_light", marker_type = 8, 
                        scale = 1.5, extra_z = 1, lifetime = 0.2)
                        self.signals_visualizator_pub.publish(signals_marker)

        # print("current traffic light: ", current_traffic_light)
        self.pub_current_traffic_light.publish(current_traffic_light)
        # print(current_traffic_light.pose.pose.position.x, current_traffic_light.pose.pose.position.y)

    # Destroy the agent

    def destroy(self):
        
        """
        Destroy (clean-up) the agent
        :return:
        """

        self.t4ac_architecture_launch.shutdown()

        pass