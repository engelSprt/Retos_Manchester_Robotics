#!/usr/bin/env python

import rospy
import numpy as np
from pid_control.msg import set_point

#Se realiza el seteo de la variables para generar la senhal senoidal
#Estos parametros incluyen la amplitud y la frecuencia
Amplitud = rospy.get_param("/setPoint_Amplitude",3.0)
Omega = rospy.get_param("/setPoint_freq",10.0)

# Setup Variables and messages to be used
ini_input = set_point()
ini_input.time = 0.0
ini_input.entrada = 0.0

#Define variables to be used
first = True
start_time = 0.0
signal = 0.0
time = 0.0

#Stop Condition
def stop():

#Setup the stop message (can be the same as the control message)
 print("Stopping")

if __name__=='__main__':

    #Initialise and Setup node
    signal_pub = rospy.Publisher("/set_point",set_point, queue_size=1)

    rospy.init_node("Set_Point_Generator")

    rate = rospy.Rate(100)

    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    print("The Set Point Generator is Running")

    start_time = rospy.get_time()

    #Run the node

    while not rospy.is_shutdown():

        time = rospy.get_time()-start_time
        #Con los parametros anteriores generamos la senhal senoidal
        signal = Amplitud*np.sin(Omega*time)

        #Publicamos la senhal al topico de /set_point

        ini_input.entrada = signal

        ini_input.time = time

        signal_pub.publish(ini_input)

        rate.sleep()
