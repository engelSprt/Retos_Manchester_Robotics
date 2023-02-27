#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import motor_output
from pid_control.msg import motor_input
from pid_control.msg import set_point
#from std_msgs.msg import Float32
#from std_msgs.msg import Float64


#Setup parameters, vriables and callback functions here (if required)
#kp = rospy.get_param("/kp",0.0)
#kd = rospy.get_param("/kd",0.0)
#ki = rospy.get_param("/ki",0.2)
#input = rospy.get_param("/input",1.0)
#output = rospy.get_param("/output",0.0)

output = 0.0
input = 0.0
time = 0.0

def callback_output(msg):
    global output
    output = msg

def callback_input(msg):
    global input
    input = msg

def callback_time(msg):
    global time
    time = msg


#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("Controller")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    rospy.Subscriber("/motor_output", motor_output, callback_output)
    rate = rospy.Rate(100)
    rospy.Subscriber("/set_point", set_point, callback_input)
    rate = rospy.Rate(100)
    rospy.Subscriber("/set_point", set_point, callback_time)
    rate = rospy.Rate(100)
    #rospy.Subscriber("/set_point", set_point, callback_input)
    #rate = rospy.Rate(100)
    #rospy.Subscriber("/set_point", set_point, callback_time)
    #rate = rospy.Rate(100)

    #loop_rate = rospy.Rate(rospy.get_param("/system_node_rate",100))
    #rospy.on_shutdown(stop)

    control_pub = rospy.Publisher("/motor_input", motor_input, queue_size=1)
    #control_time_pub = rospy.Publisher("/motor_input", motor_input, queue_size=1)


    print("The Controller is Running")

    #Run the node
    while not rospy.is_shutdown():
        #Write your code here
        #rospy.loginfo(input)
        #rospy.loginfo(time)
        control_pub.publish(input,time)
        #control_time_pub.publish(time)

        rate.sleep()
