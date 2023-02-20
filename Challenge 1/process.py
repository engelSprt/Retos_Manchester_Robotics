#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32

#Global variable to store the data from the message in the /signal topic
signal_data = 0
time_data = 0

# Example Callback Function (Hint)
def callback(msg):
    global signal_data
    signal_data = msg.data


def callback_time(msg):
    global time_data
    time_data = msg.data


if __name__=='__main__':

    #Finish configuring your node here (part of the code has already been written as a hint)
    rospy.init_node("process")
    #Subscriber example (Hint)
    rospy.Subscriber("signal", Float32, callback)
    rate = rospy.Rate(10)
    rospy.Subscriber("time", Float32, callback_time)
    rate = rospy.Rate(10)
    nueva_pub = rospy.Publisher("proc_signal",Float32, queue_size=10)
    #rate = rospy.Rate(10)


    while not rospy.is_shutdown():
        #t = (time_data)*(np.pi*2)
        t = time_data
        #half_signal = signal_data + 3
        #half_signal = half_signal * 0.5
        #t0=np.pi*2
        #t0=np.pi*2
        half_signal = ((signal_data * np.cos(10)) + np.cos(t) * np.sin(20)) * -np.cos(t)
        #half_signal = (np.sin(t) * np.cos(5)) + (np.cos(t) * np.sin(5))
        #signal_data = (np.sin(t) * np.cos(t0)) + (np.cos(t) * np.sin(t0))
        half_signal = half_signal + 3
        half_signal = half_signal * 0.5
        #half_signal = (half_signal * np.exp(-1j*t0))
        rospy.loginfo(half_signal)
        nueva_pub.publish(half_signal)

        rate.sleep()
