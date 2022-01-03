import numpy as np
import cv2
import sys
import utm

sys.path.insert(0, '/workspace/team_code/catkin_ws/src/t4ac_planning_layer/')
sys.path.insert(0, '/workspace/team_code/catkin_ws/src/t4ac_mapping_layer/')
from t4ac_map_monitor_ros.src.modules import monitor_classes
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo

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










    
    