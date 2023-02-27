#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32

t = 0
negative = False
def callback_tmn(msg):
    global t
    t = msg.data


def callback_sgn(msg):
    global t, negative
    sin = msg.data
    cos = np.sqrt(1 - pow(sin,2))
    atan = np.arctan2(cos,sin)

    if negative == True:
        y = cos * -1
    elif negative == False:
        y = cos
        
    if atan > 3:
        negative = True
    elif atan <= 0.1:
        negative = False

    signal = (y+0.5) * (np.pi/2)

    rospy.loginfo(signal)
    sign.publish(signal)

    rospy.loginfo(atan)
    tan.publish(atan)

if __name__=='__main__':
    rospy.init_node("listener")
    rospy.Subscriber("signal", Float32, callback_sgn)
    rospy.Subscriber("time", Float32, callback_tmn)
    sign=rospy.Publisher("cos",Float32, queue_size=10)
    tan=rospy.Publisher("atan",Float32, queue_size=10)
    rospy.spin()