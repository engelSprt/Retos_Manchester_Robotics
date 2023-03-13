#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import set_point
#from std_msgs.msg import Float32

# Setup Variables and messages to be used
ini_input = set_point()
ini_input.time = 0.0
ini_input.entrada = 0.0

#Define variables to be used
first = True
#start_time = 0.0
signal = 0.0
time = 0.0

def sine(amplitude, frequency):
    global time = rospy.get_time()
    y = amplitude * np.sin(2*np.pi*frequency*time)
    y = round(y, 1)
    return y

def square(amplitude, frequency):
    global time = rospy.get_time()
    y = amplitude * np.sign(np.sin(2*np.pi*frequency*time))
    return y

def step():
    y = 0
    global time = rospy.get_time()
    if (time > 0):
        y = 1
    return y

def sawtooth(amplitude, frequency):
    global time = rospy.get_time()
    period = 1/frequency
    slope = amplitude / period
    y = (time % period) * slope
    return y

if __name__=='__main__':
    pub = rospy.Publisher("/set_point", set_point, queue_size=1)
    rospy.init_node("Input")
    rate = rospy.Rate(10)

    print("The Input Generator is Running")

    while not rospy.is_shutdown():
        senal = rospy.get_param("/senal", "Parameter not found")
        amplitude = rospy.get_param("/amplitude", 1.0)
        frequency = rospy.get_param("/frequency", 1.0)

        y = 0
        if senal == "seno":
            y = sine(amplitude, frequency)
        elif senal == "cuadrada":
            y = square(amplitude, frequency)
        elif senal == "escalon":
            y = step(amplitude, frequency)
        elif senal == "sierra":
            y = sawtooth(amplitude, frequency)

        rospy.loginfo(y)

        ini_input.entrada = y
        ini_input.time = time

        pub.publish(ini_input)

        rate.sleep()
