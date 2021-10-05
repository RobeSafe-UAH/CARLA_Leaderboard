import numpy as np
import sys
import utm
sys.path.insert(0,'/workspace/team_code/catkin_ws/src/t4ac_mapping_planning/t4ac_map_builder/src')
from builder_classes import T4ac_Location

def get_input_route_list(origin, global_plan):
    """
    Return a T4ac_location points list (required by our path planner) based on a high-level route description 
    indicating the path to follow in order to reach the destination 
    """
    input_route_list = []

    for global_plan_data in global_plan:
        point_t4ac_location = T4ac_Location()

        aux = [0,0]
        u = utm.from_latlon(abs(global_plan_data[0]['lat']) , abs(global_plan_data[0]['lon']))
        u = (u[0]-origin[0],u[1]-origin[1])

        if (global_plan_data[0]['lat'] > 0):
            aux[1] = -u[1]
        else:
            aux[1] = u[1]
            pass

        if (global_plan_data[0]['lon'] < 0):
            aux[0] = -u[0]
        else:
            aux[0] = u[0]
            pass

        point_t4ac_location.x = aux[0]
        point_t4ac_location.y = -aux[1]
        point_t4ac_location.z = 0

        input_route_list.append(point_t4ac_location)

    return input_route_list

def lidar_string_to_array(lidar,whole_cloud=None):
    """
    Return the LiDAR pointcloud in numpy.array format based on a string. Every time, half (in this case) of the cloud
    is computed due to the LiDAR frequency, so if whole_cloud == True, we concatenate two consecutive pointclouds
    """
    lidar_data = np.fromstring(lidar, dtype=np.float32)
    lidar_data = np.reshape(lidar_data, (int(lidar_data.shape[0] / 4), 4))

    # we take the oposite of y axis (since in CARLA a LiDAR point is 
    # expressed in left-handed coordinate system, and ROS needs right-handed)

    lidar_data[:, 1] *= -1

    if whole_cloud:
        lidar_data = np.concatenate((self.half_cloud,lidar_data),axis=0)

    return lidar_data