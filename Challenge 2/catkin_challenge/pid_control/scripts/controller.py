#!/usr/bin/env python
import rospy
import numpy as np

#Se importan desde pid_control los archivos en donde se establecen los tipos de mensajes customizados
from pid_control.msg import motor_output
from pid_control.msg import motor_input
from pid_control.msg import set_point

#Definimos el valor para cada una de las constantes proporcional, derivativa e integral de nuestro controlador PID
kp = rospy.get_param("/kp",0.555)
kd = rospy.get_param("/kd",0.000000001)
ki = rospy.get_param("/ki",40.0)

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
last_time = 0.0
current_time = 0.0
cons_int = 0.01

#Creamos variables de tipo motor_input y motor_output, esto con el fin de utilizarlas para
#almacenar datos dentro de las funciones callback
control_msg = motor_input()
control_msg.input = 0.0
control_msg.time = 0.0
angularVelocity = motor_output()
setPoint = set_point()

#Se crean las funciones de tipo callback para obtener los datos de los topicos y nodos para su posterior uso
def callback_output(msg):
    global angularVelocity
    angularVelocity = msg

def callback_input(msg):
    global setPoint
    setPoint = msg

#Stop Condition
def stop():
#Setup the stop message (can be the same as the control message)
  print("Stopping")

if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("Controller")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Se realiza la inicializacion de Publishers and subscribers
    rospy.Subscriber("/motor_output", motor_output, callback_output)
    rospy.Subscriber("/set_point", set_point, callback_input)

    control_pub = rospy.Publisher("/motor_input", motor_input, queue_size=1)
    print("The Controller is Running")

    #Run the node

    while not rospy.is_shutdown():
        #Se obtiene un tiempo actual con la funcion de rospy.get_time()
        current_time = rospy.get_time()
        #Creamos la constante de derivada para utilizarla en la parte derivativa del controlador
        dt = current_time - last_time
        #Se establece el error actual comparando la entrada con la salida
        error = setPoint.entrada - angularVelocity.output
        #Se establece la parte derivativa del controlador
        derivative = kd * (error - error_previo) / dt
        #Se establece la parte proporcional del controlador
        proportional = kp * error
        #Creamos la constante del error integral para utilizarla en la parte integral del controlador
        Error_int += (error) * dt

        integral = ki * (cons_int + error * dt)

        # Salidad del sistema PID
        Respuesta_PID = derivative+ proportional + ki*Error_int

        control_msg.input = Respuesta_PID
        control_msg.time = rospy.get_time()-current_time

        #Publicacion de la respuesta del controlador PID
        control_pub.publish(control_msg)

        #Mediante la funcion rospy.loginfo mostramos en la consola el error del sistema para su analisis
        rospy.loginfo("Error Total: %f" % error)
        print("\n")

        last_time = rospy.get_time()

    rate.sleep()
