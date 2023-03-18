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
start_time = 0.0
signal = 0.0
#global time = 0.0

def sine(amplitude, frequency, time):
    #global time = rospy.get_time()
    y = amplitude * np.sin(frequency*time)
    #y = round(y, 1)
    return y

def square(amplitude, frequency, time):
    #global time = rospy.get_time()
    y = amplitude * np.sign(np.sin(frequency*time))
    #y = round(y, 1)
    return y

def step(time):
    y = 0
    #global time = rospy.get_time()
    if (time > 0):
        y = 1
    return y

def sawtooth(amplitude, frequency, time):
    #global time = rospy.get_time()
    period = 1/frequency
    slope = amplitude / period
    y = (time % period) * slope
    #y = round(y, 1)
    return y

if __name__=='__main__':
    rospy.init_node("Input")
    pub = rospy.Publisher("/set_point", set_point, queue_size=10) 
    #pub = rospy.Publisher("/motor_input", set_point, queue_size=1)
    rate = rospy.Rate(100)

    print("The Input Generator is Running")
    #start_time = rospy.get_time()
    y = 0
    
    while not rospy.is_shutdown():
        senal = rospy.get_param("/senal","seno")
        amplitude = rospy.get_param("/amplitude", 0.5)
        frequency = rospy.get_param("/frequency", 1.0)

        time =rospy.get_time()#-start_time
        if senal == "seno":
            y = sine(amplitude, frequency, time)
        elif senal == "cuadrada":
            y = square(amplitude, frequency, time)
        elif senal == "escalon":
            y = step(time)
        elif senal == "sierra":
            y = sawtooth(amplitude, frequency, time)
        
        y = round(y,2)
        rospy.loginfo(y)
        ini_input.entrada = y
        ini_input.time = time

        pub.publish(ini_input)

        rate.sleep()
