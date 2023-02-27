#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import set_point

# Setup Variables, parameters and messages to be used (if required)
#input = rospy.get_param("/input",1.0)
#time = rospy.get_param("/time",0.0)

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
    signal_pub = rospy.Publisher("/set_point",set_point, queue_size=1)
    #time_pub = rospy.Publisher("set_point",set_point, queue_size=1)

    print("The Set Point Generator is Running")

    input = rospy.get_param("/input")
    time = rospy.get_param("/time")

    #Run the node
    while not rospy.is_shutdown():
        #time= rospy.get_time()

        signal_pub.publish(input,time)
        #signal_pub.publish(time)
        #signal_pub.publish(set_point)

        rate.sleep()
