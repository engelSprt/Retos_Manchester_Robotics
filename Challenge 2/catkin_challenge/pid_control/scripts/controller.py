#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import motor_output
from pid_control.msg import motor_input
from pid_control.msg import set_point

#Setup parameters, vriables and callback functions here (if required)
kp = rospy.get_param("/kp",0.0)
kd = rospy.get_param("/kd",1.0)
ki = rospy.get_param("/ki",0.5)

#global set_point_nuevo, set_point_tiempo, status_nuevo, output_nuevo, output_time_nuevo, cons_int , dt
global set_point_nuevo, set_point_tiempo, status_nuevo, output_nuevo, output_time_nuevo,cons_int , dt


set_point_nuevo = 0
set_point_tiempo = 0
status_nuevo = 0
output_nuevo = 0
output_time_nuevo = 0
error = 0
error_previo = 0

cons_int = 0.01
dt = 0.01

def callback_output(msg):
    status_nuevo = msg.status
    output_nuevo = msg.output
    output_time_nuevo = msg.time

def callback_input(msg):
    set_point_nuevo = msg.input
    set_point_tiempo_nuevo = msg.time

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

    control_pub = rospy.Publisher("/motor_input", motor_input, queue_size=1)
    control_msg = motor_input()
    print("The Controller is Running")
    #Run the node
    while not rospy.is_shutdown():

        error = set_point_nuevo - output_nuevo

        derivative = kd * (error - error_previo) / dt

        proportional = kp * error

        integral =  ki * (cons_int + error * dt)
        # Salidad del sistema PID
        Respuesta_PID = derivative + proportional + integral

        error_previo = error

        control_msg.input = Respuesta_PID
        control_msg.time = rospy.get_time()

        control_pub.publish(control_msg)

        rate.sleep()