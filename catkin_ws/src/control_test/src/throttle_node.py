#!/usr/bin/env python3.8
# license removed for brevity
import rospy
from std_msgs.msg import Float64

def throttle_talker():
    pub = rospy.Publisher('/control/throttle', Float64, queue_size=10)
    rospy.init_node('throttle_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        throttle = 1.0
        pub.publish(throttle)
        rate.sleep()

if __name__ == '__main__':
    try:
        throttle_talker()
    except rospy.ROSInterruptException:
        pass