#!/usr/bin/env python
import rospy
import numpy as np

#Se importan desde pid_control los archivos en donde se establecen los tipos de mensajes customizados
from pid_control.msg import set_point
#from pid_control.msg import output
from std_msgs.msg import Float32

#Definimos el valor para cada una de las constantes proporcional, derivativa e integral de nuestro controlador PID
kp = rospy.get_param("/kp",0.05)
kd = rospy.get_param("/kd",0.01)
ki = rospy.get_param("/ki",-1.0)
sample_time = rospy.get_param("/sample_time",0.02)

#Definimos las variables globales a utilizar
global set_point_nuevo, set_point_tiempo, status_nuevo, output_nuevo, output_time_nuevo,cons_int, dt

#Inicializamos las variables para no tener errores
set_point_nuevo = 0
set_point_tiempo = 0
status_nuevo = 0
output_nuevo = 0
output_time_nuevo = 0
start_time = 0.0
last_time = 0.0
error = 0
error_previo = 0
Error_int = 0.0
current_time = 0.0
cons_int = 0.01
signal_data=0.0
time_data=0.0
###########################
#control_msg = motor_input()
#control_msg.input = 0.0
#control_msg.time = 0.0
#angularVelocity = motor_output()
#setPoint = set_point()
#######################3
#Creamos variables de tipo motor_input y motor_output, esto con el fin de utilizarlas para
#almacenar datos dentro de las funciones callback
#control_msg = 0.0
angularVelocity = 0.0
out = 0.0
#signal_data = set_point()
#signal_data.entrada = 0.0 
#signal_data.time = 0.0 
#input = motor_input()
#input.input = 0.0
#input.time = 0.0

#Se crean las funciones de tipo callback para obtener los datos de los topicos y nodos para su posterior uso
def callback_output(msg_output):
    global angularVelocity
    angularVelocity = msg_output.data

def callback_input(msg):
    global signal_data, time_data
    signal_data = msg.entrada
    time_data = msg.time
    #global setPoint
    #setPoint = msg

#Stop Condition
def stop():
#Setup the stop message (can be the same as the control message)
  print("Stopping")

if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("Controller")
    rate = rospy.Rate(100) # <<<-----------------------------------------------------------
    rospy.on_shutdown(stop)

    #Se realiza la inicializacion de Publishers and subscribers
    rospy.Subscriber("/motor_output", Float32, callback_output) #Cambiar a flotante
    rospy.Subscriber("/set_point", set_point, callback_input)

    control_pub = rospy.Publisher("/motor_input", Float32, queue_size=10)#cambiar a flotante
    print("The Controller is Running")

    #Run the node
    #current_time = rospy.get_tie()
    #
    last_time = rospy.get_time()
    #start_time = rospy.get_time()
    while not rospy.is_shutdown():
        #current_time =
        #Se obtiene un tiempo actual con la funcion de rospy.get_time()
        current_time = rospy.get_time()
        #Creamos la constante de derivada para utilizarla en la parte derivativa del controlador
        dt = current_time - last_time
        if dt >= sample_time:
            #Se establece el error actual comparando la entrada con la salida
            error = signal_data - angularVelocity
            
            #Se establece la parte derivativa del controlador
            derivative = kd * (error - error_previo) / dt
            #Se establece la parte proporcional del controlador
            proportional = kp * error
            #Creamos la constante del error integral para utilizarla en la parte integral del controlador
            Error_int += (error) * dt

            integral = ki * (cons_int + Error_int) 

            # Salidad del sistema PID
            Respuesta_PID = derivative + proportional + integral
            
            ampli=Respuesta_PID#*np.pi/2
            #control_msg.input = Respuesta_PID
            #angularVelocity.salida = Respuesta_PID
            #control_msg.time = rospy.get_time()-current_time

            #Publicacion de la respuesta del controlador PID
            #control_pub.publish(control_msg)
            #if(ampli <= 0.0):
            #   ampli = 0
            #elif ampli < -1.0:
            #    ampli = -1.0
            if out <= 0.05 and out >= -0.05:
               out = 0.0
            
            control_pub.publish(ampli)

               #Mediante la funcion rospy.loginfo mostramos en la consola el error del sistema para su analisis
            rospy.loginfo("Error Total: %f" % error)
            print("\n")

            last_time = rospy.get_time()
            
    rate.sleep()
