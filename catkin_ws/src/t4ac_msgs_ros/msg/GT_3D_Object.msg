# Techs4AgeCar project

string type                                 # Object type if it is known (car, pedestrian, truck, etc)
uint32 object_id                            # Id assign to an object
float32 alpha                               # Observation angle of object
vision_msgs/BoundingBox2D bounding_box_2D   # Object 2D bounding box
geometry_msgs/Point position                # Center of the object
geometry_msgs/Vector3 dimensions            # Object dimensions (l, w, h)
float32 rotation_z                          # Object rotation along z axis
geometry_msgs/Vector3 velocity              # Velocity of the object related to the ego vehicle