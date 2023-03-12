#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32, String
#from pwm_motor.msg import messages



def sine():

    time = rospy.get_time()
    y = np.sin(time)
    y = round(y, 1)
    
    return y


def square():
    
    
    time = rospy.get_time()
    y = np.sin(time)
    y = np.sign(y)
    
    return y 


            
def step():
    y = 0
    time = rospy.get_time()
    if (time >0):
        y = 1
        
    return y

def sawtooth():
    
    t = rospy.get_time()
    period = 2.0  # segundos
    amplitude = 1.0
    slope = amplitude / period
    y = (t % period) * slope
    
    return y



if __name__=='__main__':
    pub=rospy.Publisher("cmd_pwm",Float32, queue_size=10)
    rospy.init_node("pwm_input")
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        senal = rospy.get_param("/senal", "Parameter not found")
        #y = sine()
        #y = square()
        #y = step()
        #y = sawtooth()
        #rospy.loginfo(y)
        #pub.publish(y)
        
        y = 0
        
        if senal == "seno":
            y = sine()
        elif senal == "cuadrada":
            y = square()
        elif senal == "escalon":
            y =  step()
        elif senal == "sierra":
            y = sawtooth()
        
        rospy.loginfo(y)
        pub.publish(y)
        
        

        rate.sleep()
        
