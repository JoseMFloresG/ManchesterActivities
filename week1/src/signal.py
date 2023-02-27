#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32

if __name__=='__main__':
    pub=rospy.Publisher("signal",Float32, queue_size=10)
    tim=rospy.Publisher("time",Float32, queue_size=10)
    rospy.init_node("talker")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        t = rospy.get_time()
        sin = np.sin(t)
        rospy.loginfo(sin)
        pub.publish(sin)
        rospy.loginfo(t)
        tim.publish(t)

        rate.sleep()