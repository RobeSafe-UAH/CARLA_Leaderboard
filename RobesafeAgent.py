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

import time
import cv2
import sys
import os
import subprocess

# ROS imports

import rospy
import roslaunch
import rosgraph
import tf
import geometry_msgs
from nav_msgs.msg import Odometry
from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, NavSatStatus, PointCloud2, PointField, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from t4ac_msgs.msg import Node, RegulatoryElement, RegulatoryElementList, MonitorizedLanes
from cv_bridge import CvBridge
from std_msgs.msg import Header, String, Bool
import visualization_msgs.msg

# Math and geometry imports

import math
import numpy as np

# Custom functions imports

from generic_modules.geometric_functions import euler_to_quaternion
from generic_modules.bridge_functions import build_camera_info, build_camera_info_from_file, \
                                     cv2_to_imgmsg, image_rectification, \
                                     lidar_string_to_array, get_routeNodes, process_localization
from t4ac_global_planner_ros.src.lane_waypoint_planner import LaneWaypointPlanner
from map_parser import signal_parser
from t4ac_map_monitor_ros.src.modules import markers_module, monitor_classes
sys.path.insert(0, '/workspace/team_code/catkin_ws/src/t4ac_unified_perception_layer/src/')
from modules.monitors.monitors_functions import apply_tf, inside_lane, calculate_distance_to_nearest_object_inside_route
sys.path.insert(0, '/workspace/team_code/catkin_ws/src/t4ac_config_layer/t4ac_utils_ros/src/')
# from RobesafeAgent_ROS import publish_lidar, publish_localization, publish_cmd_vel
from RobesafeAgent_ROS import RobesafeAgentROS

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
        self.camera_parameters_path = '/workspace/team_code/generic_modules/camera_parameters/camera_parameters_/'

        # LiDAR

        self.half_cloud = []
        self.lidar_count = 0

        ## Localization

        self.pose_ekf = Odometry()
        self.ego_vehicle_yaw = 0       
        self.enabled_pose = False
        self.count_localization = 0

        ## Control

        self.Kp = 0.175
        self.Ki = 0.002
        self.error_sum = 0
        self.steer_cmd = 0
        self.speed_cmd = 0

        ## Mapping

        self.trajectory_flag = True

        ## Planning 

        self.debug_planning_route_points = True

        ### Track

        self.track = Track.MAP

        ### ROS communications

        ## Publishers

        self.pub_hdmap = rospy.Publisher('/t4ac/mapping/opendrive_data', String, queue_size=1, latch=True)
        self.pub_lidar_pointcloud = rospy.Publisher('/t4ac/perception/sensors/lidar', PointCloud2, queue_size=10)
        self.pub_radar_pointcloud = rospy.Publisher('/t4ac/perception/sensors/radar', PointCloud2, queue_size=10)

        self.pub_regulatory_elements = rospy.Publisher('/t4ac/mapping/map_monitor/regulatory_elements', RegulatoryElementList, queue_size=10, latch=True)
        self.pub_planning_routes_marker = rospy.Publisher('/t4ac/planning/route_points_marker', visualization_msgs.msg.Marker, queue_size=10)
        self.pub_route_goal = rospy.Publisher('/t4ac/planning/goal',geometry_msgs.msg.PoseStamped, queue_size=10,latch=True)
        
        ## Subscribers

        # self.sub_cmd_vel = rospy.Subscriber('/t4ac/control/cmd_vel', CarControl, self.read_cmd_vel_callback)
        self.sub_pose_ekf = rospy.Subscriber('/t4ac/localization/pose', Odometry, self.read_pose_callback)

        flag_processed_data_topic = "/t4ac/perception/flag_processed_data"
        self.sub_flag_processed_data = rospy.Subscriber(flag_processed_data_topic, Header, self.flag_processed_data_callback)

        ## Launch the architecture

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        try:
            rosgraph.Master('/rostopic').getPid()
        except:
            launch = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_files=[], is_core=True)
            launch.start()

        # T4AC roslaunch
        
        self.t4ac_architecture_launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/workspace/team_code/catkin_ws/src/t4ac_config_layer/t4ac_utils_ros/launch/t4ac_config.launch"])
        self.t4ac_architecture_launch.start()
        time.sleep(2)

        self.RobesafeAgentROS = RobesafeAgentROS()

        rospy.loginfo("Techs4AgeCar architecture started")

        ## Init the node

        rospy.init_node('robesafe_agent')#, anonymous=True, disable_signals=True)

        ## Frames

        self.map_frame = rospy.get_param('/t4ac/frames/map')
        self.base_link_frame = rospy.get_param('/t4ac/frames/base_link')
        self.lidar_frame = rospy.get_param('/t4ac/frames/lidar')
        self.radar_frame = rospy.get_param('/t4ac/frames/radar')

        ## Transforms 

        self.tf_lidar2map = np.zeros((4,4))

        # Local position of front bumper center frame with respect to base_link frame (Local point)

        self.base_link_to_front_bumper_center_tf = rospy.get_param('/t4ac/tf/base_link_to_front_bumper_center_tf')
        self.front_bumper_center_local_position = Node(self.base_link_to_front_bumper_center_tf[0],
                                                       self.base_link_to_front_bumper_center_tf[1],
                                                       self.base_link_to_front_bumper_center_tf[2])
        self.front_bumper_center_global_position = Node(50000,50000,50000)

        self.listener = tf.TransformListener()

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
        self.radar_position = rospy.get_param('t4ac/tf/base_link_to_radar_tf')
        self.xradar, self.yradar, self.zradar = self.radar_position[:3]

        parameters = rospy.get_param_names()

        cameras_id = []
        parameters = rospy.get_param_names()
        for parameter in parameters:
            if "t4ac/sensors" in parameter and "camera" in parameter:
                camera_id = parameter.split('/')[-1]
                cameras_id.append(camera_id)

        self.cameras_parameters = []
        for _, camera_id in enumerate(cameras_id):
            """
            camera_id = center, right, left, rear
            camera_parameters = width, height, fov
            """

            camera_parameters = rospy.get_param('/t4ac/sensors/camera/' + str(camera_id)) # width, height, fov
            camera_position = rospy.get_param('/t4ac/tf/base_link_to_camera_' + str(camera_id) + '_tf') # x,y,z,roll,pitch,yaw
            camera_parameters_path = self.camera_parameters_path + str(camera_parameters[0]) + '_' + str(camera_parameters[1]) + '_' + str(camera_parameters[2]) + '/'

            camera_dict = dict({
                                'id': camera_id,
                                'width': camera_parameters[0],
                                'height': camera_parameters[1],
                                'fov': camera_parameters[2],
                                'fx': camera_parameters[0] / (2.0 * math.tan(camera_parameters[2] * math.pi / 360.0)),
                                'fy': camera_parameters[0] / (2.0 * math.tan(camera_parameters[2] * math.pi / 360.0)), # fx = fy
                                'cx': camera_parameters[0] / 2,
                                'cy': camera_parameters[1] / 2,
                                'camera_parameters_path': camera_parameters_path,
                                'image_raw_pub': rospy.Publisher('/t4ac/perception/sensors/camera/' + camera_id + '/image_raw', Image, queue_size=20, latch=True), # 100
                                'camera_info_pub': rospy.Publisher('/t4ac/perception/sensors/camera/' + camera_id + '/camera_info', CameraInfo, queue_size=20, latch=True), # 100
                                # 'image_rect_pub': rospy.Publisher('/t4ac/perception/sensors/camera/' + camera_id + '/image_rect', Image, queue_size=20), # 100
                                # 'camera_info_rect_pub': rospy.Publisher('/t4ac/perception/sensors/camera/' + camera_id + '/rect/camera_info', CameraInfo, queue_size=20), # 100
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
        
        self.previous_simulation_iteration_stamp = rospy.Time.now()
        time.sleep(1)
        self.current_simulation_iteration_stamp = rospy.Time.now() # Current simulation iteration stamp must be 
                                                                   # higher to previous simulation iteration
                                                                   # to run step
        rospy.set_param("/t4ac/perception/flag_processed_data", flag_processed_data_topic)

        print("\033[1;31m"+"End init configuration: "+'\033[0;m')

    # Specify your sensors

    def sensors(self):
        sensors = []

        for _, camera in enumerate(self.cameras_parameters):
            sensors.append({'type': 'sensor.camera.rgb', 'x': camera['x'], 'y': camera['y'], 'z': camera['z'], 
                            'roll': 0.0, 'pitch': 0.0, 'yaw': camera['yaw'], 'width': camera['width'], 
                            'height': camera['height'], 'fov': camera['fov'], 'id': camera['id']}) 

        sensors.append({'type': 'sensor.lidar.ray_cast', 'x': self.xlidar, 'y': self.ylidar, 'z': self.zlidar, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'id': 'LiDAR'})
        sensors.append({'type': 'sensor.other.gnss', 'x': self.xgnss, 'y': self.ygnss, 'z': self.zgnss, 'id': 'GNSS'})
        sensors.append({'type': 'sensor.other.imu', 'x': self.xgnss, 'y': self.ygnss, 'z': self.zgnss, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'id': 'IMU'})
        sensors.append({'type': 'sensor.opendrive_map', 'reading_frequency': 1, 'id': 'OpenDRIVE'})
        sensors.append({'type': 'sensor.speedometer',  'reading_frequency': 20, 'id': 'Speed'})
        sensors.append({'type': 'sensor.other.radar', 'x': self.xradar, 'y': self.yradar, 'z': self.zradar, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'fov': 90, 'id': 'RADAR'})
                    
        return sensors

    # Run step function

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation. Gather the sensor information
        """
        current_ros_time = rospy.Time.now()

        while (self.current_simulation_iteration_stamp <= self.previous_simulation_iteration_stamp):
            continue
        self.previous_simulation_iteration_stamp = self.current_simulation_iteration_stamp

        # Get sensor data

        if (self.trajectory_flag) and ("OpenDRIVE" in list(input_data.keys())):
            hd_map = (input_data['OpenDRIVE'][1])
            distance_among_waypoints = rospy.get_param('/t4ac/map_parameters/distance_among_waypoints')
            rospy.set_param('/t4ac/map_parameters/map_data', hd_map['opendrive'])
            self.LWP = LaneWaypointPlanner(hd_map['opendrive'],1)
            self.map_name = self.LWP.map_name
            rospy.set_param('/t4ac/map_parameters/map_name', self.map_name)
            self.route = self.LWP.calculate_waypoint_route_multiple(distance_among_waypoints, self._global_plan_world_coord, 1)
            
            # Every element of self._global_plan_world_coord is a tuple with the corresponding wp in global coordinates
            # and a flag to indicate if lane change is required
            last_wp = self._global_plan_world_coord[-1][0]
            goal = geometry_msgs.msg.PoseStamped()
            goal.header.stamp = current_ros_time
            goal.header.frame_id = self.map_frame
            goal.pose.position.x = last_wp.location.x
            goal.pose.position.y = last_wp.location.y
            goal.pose.position.z = last_wp.location.z
            self.pub_route_goal.publish(goal) # To start the Petri Nets node (equivalent to introduce
            # a goal in RVIZ, required to evaluate the other states)

            geometry_msgs.msg.PoseStamped
            if self.debug_planning_route_points:
                routeNodes = get_routeNodes(self.route)
                waypoints_marker = markers_module.get_nodes(routeNodes, [0,0,1], "2", 8, 0.5, 1, 0)
                self.pub_planning_routes_marker.publish(waypoints_marker)
            
            traffic_lights_signals, stop_signals = signal_parser.parse_regElems(self.LWP.map_object.roads)

            # Publish as t4ac_msgs.msg.RegulatoryElements

            regulatory_elements = RegulatoryElementList()

            for signal in traffic_lights_signals:
                regelem = RegulatoryElement()
                regelem.type = signal['name']
                regelem.id = signal['id']
                regelem.global_location.pose.position.x = signal['x']
                regelem.global_location.pose.position.y = signal['y']
                regelem.global_location.pose.position.z = signal['z']
                [qx, qy, qz, qw] = euler_to_quaternion(0, 0, signal['yaw'])
                regelem.global_location.pose.orientation.x = qx
                regelem.global_location.pose.orientation.y = qy
                regelem.global_location.pose.orientation.z = qz
                regelem.global_location.pose.orientation.w = qw
                regulatory_elements.reg_elems.append(regelem)

            for signal in stop_signals:
                regelem = RegulatoryElement()
                regelem.type = signal['name']
                regelem.id = signal['id']
                regelem.global_location.pose.position.x = signal['x']
                regelem.global_location.pose.position.y = signal['y']
                regelem.global_location.pose.position.z = signal['z']
                regulatory_elements.reg_elems.append(regelem)

            self.pub_regulatory_elements.publish(regulatory_elements)

            self.trajectory_flag = False

        actual_speed = (input_data['Speed'][1])['speed']
        gnss = (input_data['GNSS'][1])
        imu = (input_data['IMU'][1])
        lidar = (input_data['LiDAR'][1])
        radar = (input_data['RADAR'][1])

        # Publish until control has started   

        if actual_speed < 0.2: # TODO: Improve this. Publish until the corresponding subscribers have
            # subscribe, not hardcoding a minimum speed
            self.LWP.publish_waypoints(self.route)

        # Localization Layer   
        enabled_pose_msg = Bool()     
        gnss_msg = NavSatFix()
        gnss_pose_msg = Odometry()
        speed_msg = TwistWithCovarianceStamped()
        imu_msg = Imu()

        gnss_msg, gnss_pose_msg, speed_msg, imu_msg, self.ego_vehicle_yaw, self.enabled_pose, self.count_localization = process_localization(gnss, imu, actual_speed, current_ros_time, self.map_frame, self.base_link_frame, self.enabled_pose, self.count_localization)
        enabled_pose_msg.data = self.enabled_pose

        self.RobesafeAgentROS.publish_localization(enabled_pose_msg, gnss_msg, gnss_pose_msg, speed_msg, imu_msg)

        # Callbacks

        cameras = []
        for camera in self.cameras_parameters:
            cameras.append(input_data[camera['id']][1]) 
        self.cameras_callback(cameras, current_ros_time)

        self.lidar_callback(lidar, current_ros_time)
        self.radar_callback(radar, current_ros_time)
        control = self.control_callback(actual_speed)

        return control 

    # Callbacks

    def flag_processed_data_callback(self, flag_processed_data_msg):
        """
        """

        self.current_simulation_iteration_stamp = flag_processed_data_msg.stamp

    def lidar_callback(self, lidar, current_ros_time):
        """
        Return the LiDAR pointcloud as a sensor_msgs.PointCloud2 ROS message based on a string that contains
        the LiDAR information
        """

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
            # self.pub_lidar_pointcloud.publish(point_cloud_msg)
            self.RobesafeAgentROS.publish_lidar(point_cloud_msg)
        else:
            self.half_cloud = lidar_string_to_array(lidar)
    
    def radar_callback(self, radar_data, current_ros_time):

        # New header with current ROS information: time and frame link
        header = Header()
        header.stamp = current_ros_time
        header.frame_id = self.radar_frame

        # print("-" * 75)
        # print(f"'\033[93m'INPUT is {radar_data} and its type is {type(radar_data)}. Its shape is {len(radar_data)}'\033[0m'")


        # CARLA Radar Raw Data format
        # Source: https://carla.readthedocs.io/en/latest/python_api/#carlaradardetection

        # vel, azimuth, altitude, depth

        # [:,0]: Depth [m]
        # [:,1]: Azimuth [rad]
        # [:,2]: Altitude [rad]
        # [:,3]: Vel [m/s]
        # Adaptation to ROS Bridge taken from: 
        #   https://github.com/carla-simulator/ros-bridge/blob/master/carla_ros_bridge/src/carla_ros_bridge/radar.py
        #   https://carla.readthedocs.io/en/latest/ref_sensors/#radar-sensor

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='Range', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='Velocity', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='AzimuthAngle', offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='ElevationAngle', offset=28, datatype=PointField.FLOAT32, count=1)]
      
        points = []
        for detection in radar_data:
            points.append([detection[0] * np.cos(detection[2]) * np.cos(-detection[1]),     # X
                           detection[0] * np.sin(-detection[2]) * np.cos(detection[1]),     # Y
                           detection[0] * np.sin(detection[1]),                             # Z
                           detection[0],                                                    # Range (depth)
                           detection[3],                                                    # Velocity
                           detection[2],                                                    # Azimuth
                           detection[1]])                                                   # Altitude

        # print(f"'\033[93m'OUTPUT is {points} and its type is {type(points)}. Its shape is {len(points)}'\033[0m'")

        radar_msg = create_cloud(header, fields, points)

        self.RobesafeAgentROS.publish_radar(radar_msg)
        
    def cameras_callback(self, cameras, current_ros_time):
        """
        Return the information of the corresponding camera as a sensor_msgs.Image ROS message based on a string 
        that contains the camera information
        """
        for index_camera, raw_image in enumerate(cameras):
            raw_image = cv2_to_imgmsg(raw_image, self.encoding)
            raw_image.header.stamp = current_ros_time
            raw_image.header.frame_id = self.cameras_parameters[index_camera]['frame']    
            self.cameras_parameters[index_camera]['image_raw_pub'].publish(raw_image)

    def rectified_cameras_callback(self, cameras, current_ros_time):
        """
        Return the information of the correspondin camera as a sensor_msgs.Image ROS message based on a string 
        that contains the camera information
        """
        start = time.time()
        for index_camera, raw_image in enumerate(cameras):
            raw_image_K = build_camera_info(raw_image.shape[1], raw_image.shape[0], 
                                            self.cameras_parameters[index_camera]['fx'],
                                            self.cameras_parameters[index_camera]['fy'],
                                            self.cameras_parameters[index_camera]['camera_position_3D'][0,0],
                                            self.cameras_parameters[index_camera]['camera_position_3D'][0,1], 
                                            current_ros_time, self.cameras_parameters[index_camera]['frame'],
                                            distorted_image=True)
        
            # Rectify the image

            cv_image = self.bridge.imgmsg_to_cv2(raw_image, desired_encoding='passthrough')

            if self.calibrate_camera:
                cv2.imwrite("/workspace/team_code/generic_modules/camera_parameters/distorted_image_" + str(self.cameras_parameters[index_camera]['id']) + ".png", cv_image)

            camera_parameters_path = self.cameras_parameters[index_camera]['camera_parameters_path']
            start_rect = time.time()
            roi_rectified_image = image_rectification(raw_image, raw_image_K, camera_parameters_path)
            end_rect = time.time()
            
            start_roi = time.time()
            roi_rectified_image = cv2_to_imgmsg(raw_image, self.encoding)
            end_roi = time.time()
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

            raw_image = cv2_to_imgmsg(raw_image, self.encoding) # to publish as ROS topic
            raw_image.header.stamp = current_ros_time
            raw_image.header.frame_id = self.cameras_parameters[index_camera]['frame']
            
            self.cameras_parameters[index_camera]['image_raw_pub'].publish(raw_image)
            # self.cameras_parameters[index_camera]['camera_info_pub'](raw_image_info) # TODO: Calculate the rectification matrix based on the distortion coefficients
            self.cameras_parameters[index_camera]['image_rect_pub'].publish(roi_rectified_image)
            self.cameras_parameters[index_camera]['camera_info_rect_pub'].publish(rectified_image_info)
        end = time.time()

 
    def control_callback(self, actual_speed):
        """
        Return the current state of the vehicle regarding the control layer
        """
        # error_speed = self.speed_cmd - actual_speed  # distance away from setpoint [m/s]

        self.speed_cmd, self.steer_cmd = self.RobesafeAgentROS.publish_cmd_vel()
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
        """
        """
        self.pose_ekf = data
        
    # Destroy the agent

    def destroy(self):
        
        """
        Destroy (clean-up) the agent
        :return:
        """

        self.t4ac_architecture_launch.shutdown()

        self.ego_vehicle_yaw = 0       
        self.enabled_pose = False
        self.count_localization = 0

        pass