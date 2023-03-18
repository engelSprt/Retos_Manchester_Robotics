#!/usr/bin/env python
import rospy
import numpy as np
#from pid_control.msg import set_point
from std_msgs.msg import Float32

start_time = 0.0
signal = 0.0
time = 0.0

if __name__=='__main__':
    rospy.init_node("Generator")
    rate = rospy.Rate(10)
    
    signal_pub = rospy.Publisher("/motor_input",Float32, queue_size=10)
    #time_pub = rospy.Publisher("/time",Float32, queue_size=10)
    
    Amplitud = 1
    Omega = 1
    start_time = rospy.get_time()
    print("The Input Generator is Running")
    
    while not rospy.is_shutdown():
        time = rospy.get_time()-start_time
        function = Amplitud*np.sin(Omega*time)
        rospy.loginfo(function)
        signal_pub.publish(function)
        #time_pub.publish(time)
        rate.sleep()