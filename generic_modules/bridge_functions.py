import numpy as np
import cv2
import sys
import utm
import math

sys.path.insert(0, '/workspace/team_code/catkin_ws/src/t4ac_planning_layer/')
sys.path.insert(0, '/workspace/team_code/catkin_ws/src/t4ac_mapping_layer/')
from t4ac_map_monitor_ros.src.modules import monitor_classes
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo, NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from generic_modules.geometric_functions import euler_to_quaternion

def lidar_string_to_array(lidar, half_cloud=None, whole_cloud=None):
    """
    Return the LiDAR pointcloud in numpy.array format based on a string. Every time, half (in this case) of the cloud
    is computed due to the LiDAR frequency, so if whole_cloud == True, we concatenate two consecutive pointclouds
    """
    lidar_data = np.fromstring(lidar, dtype=np.float32)
    lidar_data = np.reshape(lidar_data, (int(lidar_data.shape[0] / 4), 4))

    # We take the oposite of y axis (since in CARLA a LiDAR point is 
    # expressed in left-handed coordinate system, and ROS needs right-handed)

    lidar_data[:, 1] *= -1

    if whole_cloud:
        lidar_data = np.concatenate((half_cloud,lidar_data),axis=0)

    return lidar_data

def cv2_to_imgmsg(cvim, encoding = "passthrough"):
    """
    Convert an OpenCV :cpp:type:`cv::Mat` type to a ROS sensor_msgs::Image message.
    :param cvim:      An OpenCV :cpp:type:`cv::Mat`
    :param encoding:  The encoding of the image data, one of the following strings:
        * ``"passthrough"``
        * one of the standard strings in sensor_msgs/image_encodings.h
    :rtype:           A sensor_msgs.msg.Image message
    :raises CvBridgeError: when the ``cvim`` has a type that is incompatible with ``encoding``
    If encoding is ``"passthrough"``, then the message has the same encoding as the image's OpenCV type.
    Otherwise desired_encoding must be one of the standard image encodings
    This function returns a sensor_msgs::Image message on success, or raises :exc:`cv_bridge.CvBridgeError` on failure.
    """

    if not isinstance(cvim, (np.ndarray, np.generic)):
        raise TypeError('Your input type is not a numpy array')
    img_msg = Image()
    img_msg.height = cvim.shape[0]
    img_msg.width = cvim.shape[1]

    if len(cvim.shape) < 3:
        cv_type = 'mono8' 
    else:
        cv_type = 'bgr8'
    if encoding == "passthrough":
        img_msg.encoding = cv_type
    else:
        img_msg.encoding = encoding

    if cvim.dtype.byteorder == '>':
        img_msg.is_bigendian = True
    img_msg.data = cvim.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height

    return img_msg

def build_camera_info(width, height, f_x, f_y, x, y, current_ros_time, frame_id, distorted_image=None):
    """
    Private function to compute camera info
    camera info doesn't change over time
    """
    camera_info = CameraInfo()
    camera_info.header.stamp = current_ros_time
    camera_info.header.frame_id = frame_id
    camera_info.width = width
    camera_info.height = height
    camera_info.distortion_model = 'plumb_bob'
    cx = camera_info.width / 2.0
    cy = camera_info.height / 2.0
    fx = f_x
    fy = f_y
    camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]

    if not distorted_image:
        camera_info.D = [0, 0, 0, 0, 0]
        camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        camera_info.P = [fx, 0, cx, x, 0, fy, cy, y, 0, 0, 1.0, 0]

        return camera_info
    else:
        return np.array([camera_info.K]).reshape(3,3) # Only return intrinsic parameters

def build_camera_info_from_file(frame, x_pos, y_pos, current_ros_time, camera_parameters_path='/workspace/team_code/generic_modules/camera_parameters/'):
    """
    Private function to compute camera info
    camera info doesn't change over time
    """

    x = x_pos
    y = y_pos

    K = np.loadtxt(camera_parameters_path+'K.txt')
    fx = K[0,0]
    fy = K[1,1]
    cx = K[0,2]
    cy = K[1,2]

    # print("x, y: ", x, y)
    # print("fx, fy, cx, cy: ", fx, fy, cx, cy)

    roi = np.loadtxt(camera_parameters_path+'roi.txt')
    xtl,ytl,width,height = roi
    width = int(width)
    height = int(height)

    camera_info = CameraInfo()
    camera_info.header.stamp = current_ros_time
    camera_info.header.frame_id = frame
    camera_info.width = width
    camera_info.height = height
    camera_info.distortion_model = 'plumb_bob'

    camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    camera_info.D = [0, 0, 0, 0, 0]
    camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
    camera_info.P = [fx, 0, cx, x, 0, fy, cy, y, 0, 0, 1.0, 0]

    return camera_info

def image_rectification(distorted_image, camera_parameters_path='/workspace/team_code/generic_modules/camera_parameters/'):
    """
    """

    K_distorted = np.loadtxt(camera_parameters_path+'K_original.txt') # Load your original K matrix
    D = np.loadtxt(camera_parameters_path+'D.txt') # Load the distortion coefficients of your original image
    roi = np.loadtxt(camera_parameters_path+'roi.txt').astype(np.int64) # Load ROI dimensions

    h_dist, w_dist = distorted_image.shape[:2]

    x,y,w,h = roi

    dst = cv2.undistort(distorted_image, K_distorted, D, None) # Undistort
    dst = dst[y:y+h, x:x+w]
    return dst

def get_routeNodes(route):
    """
    Returns the route in Node3D format to visualize it on RVIZ
    """

    nodes = []

    for waypoint in route:
        node = monitor_classes.Node3D()
        node.x = waypoint.transform.location.x
        node.y = -waypoint.transform.location.y
        node.z = 0
        nodes.append(node)
    return nodes

def process_localization(gnss, imu, actual_speed, current_ros_time, map_frame, base_link_frame, enabled_pose, count_localization):
    """
    Return UTM position (x,y,z) and orientation of the ego-vehicle as a nav_msgs.Odometry ROS message based on the
    gnss information (WGS84) and imu (to compute the orientation)
        GNSS    ->  latitude =  gnss[0] ; longitude = gnss[1] ; altitude = gnss[2]
        IMU     ->  accelerometer.x = imu[0] ; accelerometer.y = imu[1] ; accelerometer.z = imu[2] ; 
                    gyroscope.x = imu[3]  ;  gyroscope.y = imu[4]  ;  gyroscope.z = imu[5]  ;  compass = imu[6]
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
    ##----------- Errores en las medidas:  ------------------
    ##   x_gnss = ±  1.75m      y_gnss = ± 1.75m
    ##   yaw_compass = ± 0.00     
    ##   actual_speed = ± 0.00 m/s      
    ##   x_acc_imu = ± 0.0025    y_acc_imu = ±  0.0025   z_vel_imu = ± 0.0025   
    ##   IMU: cuidado con espurios muy muy pocos con aceleraciones mayores de ± 10000 m/s2
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

    gnss_pose_msg = Odometry()
    gnss_pose_msg.header.frame_id = map_frame
    gnss_pose_msg.child_frame_id = base_link_frame
    gnss_pose_msg.header.stamp = current_ros_time
    gnss_pose_msg.pose.pose.position.x = x
    gnss_pose_msg.pose.pose.position.y = y
    gnss_pose_msg.pose.pose.position.z = 0
    gnss_pose_msg.pose.pose.orientation.x = qx
    gnss_pose_msg.pose.pose.orientation.y = qy
    gnss_pose_msg.pose.pose.orientation.z = qz
    gnss_pose_msg.pose.pose.orientation.w = qw

    speed_msg = TwistWithCovarianceStamped()
    speed_msg.header.frame_id = base_link_frame
    speed_msg.header.stamp = current_ros_time
    speed_msg.twist.twist.linear.x = actual_speed
    speed_msg.twist.twist.linear.y = 0
    
    imu_msg = Imu()
    imu_msg.header.frame_id = base_link_frame
    imu_msg.header.stamp = current_ros_time
    imu_msg.orientation.x = qx
    imu_msg.orientation.y = qy
    imu_msg.orientation.z = qz
    imu_msg.orientation.w = qw
    imu_msg.angular_velocity.x = 0
    imu_msg.angular_velocity.y = 0
    imu_msg.angular_velocity.z = -imu[5]  ##Carla tiene los ejes de coordenadas de todo (mapa, sensores...) con la Y en sentido opuesto
    imu_msg.linear_acceleration.x = imu[0]
    imu_msg.linear_acceleration.y = 0   ##Carla tiene los ejes de coordenadas de todo (mapa, sensores...) con la Y en sentido opuesto
    imu_msg.linear_acceleration.z = 0

    if not enabled_pose:
        gnss_translation_error = 0.0001 # [m]  ##Para converger rápidamente 
        count_localization += 1
        if (count_localization >= 50):
            enabled_pose = True
    else:
        if (actual_speed > 0.25):
            gnss_translation_error = 2.5 # [m]  
        else:
            gnss_translation_error = 50.0 # [m]  ##Para evitar oscilaciones en parado
            
    #gnss_translation_error = 2.5 # [m] 0.5   5000000000: no la tiene en cuenta  ;  50: la filtra bien pero es lento  ; 0.05 reduzco error frenada (con error gnss 0)
    gnss_rotation_error = 0.001 # [rad] 0.001

    x_speed_error = 4.5 # [m/s]
    y_speed_error = 0.001 # [m/s]
    
    imu_gyroscope_error = 4.5
    imu_accelerometer_error = 4.5

    gnss_pose_msg.pose.covariance = np.diag([gnss_translation_error, gnss_translation_error, 0, 0, 0, gnss_rotation_error]).ravel()
    speed_msg.twist.covariance = np.diag([x_speed_error, y_speed_error, 0, 0, 0, 0]).ravel()
    imu_msg.angular_velocity_covariance = np.diag([0, 0, imu_gyroscope_error]).ravel()
    imu_msg.linear_acceleration_covariance = np.diag([imu_accelerometer_error, imu_accelerometer_error, 0]).ravel()

    return gnss_msg, gnss_pose_msg, speed_msg, imu_msg, yaw, enabled_pose, count_localization
