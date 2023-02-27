#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import set_point

# Setup Variables, parameters and messages to be used (if required)
# Mando llamar l
imp = set_point
imp.time = 0
imp.signal = 0

#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("Set_Point_Generator")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    sig = rospy.Publisher("signal", Float32, queue_size=10)
    tim = rospy.Publisher("time", Floaat32, queue_size=10)

    print("The Set Point Genertor is Running")

    #Run the node
    while not rospy.is_shutdown():

        #Write your code here
        imp.time = rospy.get_time()
        imp.signal = np.sin(t)

        rospy.loginfo(imp.time)
        tim.publish(imp.time)

        rospy.loginfo(imp.signal)
        sig.publish(imp.signal)

        rate.sleep()