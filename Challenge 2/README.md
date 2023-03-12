# CHALLENGE 2

<p align="center">
  <img src="https://github.com/engelSprt/Retos_Manchester_Robotics/blob/main/Challenge%201/Imagenes/tecnologico-de-monterrey-blue.png" />
</p>


**<p align="center">Instituto Tecnológico y de Estudios Superiores de Monterrey</p>**
<p align="center">TE3001B.101.</p>
<p align="center">Fundamentación de robótica (Gpo 101)</p>
<p align="center">Semestre: febrero - junio 2023.</p>
<p align="center">Challenge 2</p>
<p align="center"> Integrantes:</p>
<p align="center">Fredy Yahir Canseco Santos		A01735589</p>
<p align="center">José Ángel Ramírez Ramírez		A01735529</p>
<p align="center">Daniel Ruán Aguilar			A01731921</p>
<p align="center">Fecha de entrega: 26 de febrero del 2023</p>


## Resumen

Nuevamente ponemos en práctica los conceptos de las sesiones y seguimos viendo la comunicación entre nodos pero con más de complejidad. 

Un nodo simulará un sistema de primer orden, que será la aproximación a un motor de CC.

Otro nodo se encargará de ser el controlador de motor.

La entrada y la salida serán mensajes personalizados creados específicamente para este sistema, dichos mensajes se usarán para comunicarse con el sistema.

Para este reto se empelearán los siguientes elementos aprendidos en la semana del material de autoestudio brindado por MCR2:
* ROS Namespaces
* ROS Parameters files
* ROS Custom Messages
* ROS Topics


## Objetivos
El reto de esta semana esta destinado a repasar los conceptos introducidos en las sesiones anteriores a través de la siguiente actividad.

* Consiste en crear un controlador para un sistema simple de primer orden en ROS.
* El sistema representa el comportamiento dinámico de un motor DC.
* El controlador será un “PID” (Proporcional Integral Derivativo).
* Al final se graficarán los resultados empleando rqt_graph y rqt_plot con un archivo.launch previamente creado.
  

## Introducción
La robótica es la rama de la ingeniería mecánica, electrónica e informática que se especializa en el diseño y construcción de robots que realizan trabajos, generalmente en la sustitución de la mano de obra humana.

La evolución de la robótica se ha visto desarrollada en los últimos años, sobre todo con la automatización y la industria 4.0, debido a que genera beneficios económicos significativos, por lo que manejar la herramienta R.O.S. es una necesidad de nuestra carrera profesional.

Continuando con el aprendizaje de las herramientas de ROS tenemos los siguientes elementos importantes:
<p align="center">
  <img src="https://github.com/engelSprt/Retos_Manchester_Robotics/blob/main/Challenge%202/images/2_ROS_Practicalities..png" />
</p>

## Solución del problema
Para la solución de este reto se cuentan con tres archivos codificados en lenguaje Python, llamados: system, controller y set_point. Además del archivo.launch para 
ejecutar todos los nodos. Se comenzará por describir la funcionalidad de cada uno de ellos en el siguiente apartado:

### System.py

Este archivo contiene la simulación de un motor de Corriente directa con encoder, dicho sistema responde a cualquier señal y es el responsable de que se tenga que aplicar un controlador, dicho sistema es uno de primer orden.

`````python
#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import motor_output
from pid_control.msg import motor_input
#from std_msgs.msg import Float32
#from std_msgs.msg import Float64

class SimpleSystem:

  def __init__(self):

    #Set the parameters of the system
    self.sample_time = rospy.get_param("/system_sample_time",0.002)
    self.max_speed = rospy.get_param("/system_max_speed",13)
    self.min_input = rospy.get_param("/system_min_input",0.05)
    self.param_K = rospy.get_param("/system_param_K",13.1)
    self.param_T = rospy.get_param("/system_param_T",0.04)
    self.init_conditions = rospy.get_param("/system_initial_cond",0.0)

    # Setup Variables to be used
    self.first = True
    self.start_time = 0.0
    self.current_time = 0.0
    self.last_time = 0.0
    self.proc_output = 0.0

    # Declare the input Message
    self.Input = motor_input()
    self.Input.input = 0.0
    self.Input.time = 0.0


    # Declare the  process output message
    self.output = motor_output()
    self.output.output= self.init_conditions
    self.output.time = rospy.get_time()
    self.MotorStatus(self.init_conditions)


    # Setup the Subscribers
    rospy.Subscriber("/motor_input",motor_input,self.input_callback)

    #Setup de publishers
    self.state_pub = rospy.Publisher("/motor_output", motor_output, queue_size=1)

  #Define the callback functions
  def input_callback(self,msg):
    self.Input = msg

  #Define the main RUN function
  def run (self):
    #Variable setup
    if self.first == True:
      self.start_time = rospy.get_time()
      self.last_time = rospy.get_time()
      self.current_time = rospy.get_time()
      self.first = False
  #System
    else:
      #Define sampling time
      self.current_time = rospy.get_time()
      dt = self.current_time - self.last_time

      #Dynamical System Simulation
      if dt >= self.sample_time:
        #Dead-Zone
        if(abs(self.Input.input)<=self.min_input):
          self.proc_output+= (-1.0/self.param_T * self.proc_output + self.param_K/self.param_T * 0.0) * dt
        #Saturation
        elif (((-1.0/self.param_T * self.proc_output + self.param_K/self.param_T * self.Input.input)>0.0 and self.proc_output> self.max_speed)or ((-1.0/self.param_T * self.proc_output + self.param_K/self.param_T * self.Input.input)<0.0 and self.proc_output< -self.max_speed)):
          self.proc_output+= (-1.0/self.param_T * self.proc_output + self.param_K/self.param_T * ((1/self.param_K)*self.proc_output)) * dt
        #Dynamic System
        else:
          self.proc_output += dt*((-1.0/self.param_T) * self.proc_output + (self.param_K/self.param_T) * self.Input.input)

        #Message to publish
        self.output.output= self.proc_output
        self.output.time = rospy.get_time() - self.start_time
        self.MotorStatus(self.proc_output)
        #Publish message
        self.state_pub.publish(self.output)

        self.last_time = rospy.get_time()

      #else:
      #self.state_pub.publish(self.output)

  # Motor Status Function
  def MotorStatus(self,speed):
    if (abs(speed)<=abs(self.param_K*self.Input.input*0.8) and abs(self.Input.input)<=self.min_input):
      self.output.status = "Motor Not Turning"
    elif (abs(speed)>=self.max_speed):
      self.output.status = "Motor Max Speed"
    else:
      self.output.status = "Motor Turning"



 #Stop Condition
  def stop(self):
  #Setup the stop message (can be the same as the control message)
    print("Stopping")
    self.output.output= 0.0
    self.output.time = rospy.get_time() - self.start_time
    self.output.status = "Motor Not Turning"
    self.state_pub.publish(self.output)
    total_time = rospy.get_time()-self.start_time
    rospy.loginfo("Total Simulation Time = %f" % total_time)



if __name__=='__main__':

 #Initialise and Setup node
 rospy.init_node("Motor_Sim")
 System = SimpleSystem()

 # Configure the Node
 loop_rate = rospy.Rate(rospy.get_param("/system_node_rate",1000))
 rospy.on_shutdown(System.stop)

 print("The Motor is Running")
 try:
  #Run the node
  while not rospy.is_shutdown():
   System.run()
   loop_rate.sleep()

 except rospy.ROSInterruptException:
  pass

`````

### Controller.py

Este archivo contiene el controlador programado en código Python, dicho controlador es un PID y a continuación se describe el cómo fue posible ajustar cada una de las constantes y el código comentado:

la metodología empleada para encontrar las constantes del PID fue mendiante el ajuste manual, que es un proceso iterativo que consiste en ajustar manualmente las constantes del controlador hasta lograr un rendimiento óptimo del sistema. El proceso comienza ajustando la ganancia proporcional (Kp) del controlador. Se aumenta gradualmente Kp hasta que se observa una respuesta del sistema, y se observa si esta respuesta se acerca a la respuesta deseada. Si la respuesta no es la esperada, se ajusta Kp de nuevo, y se sigue probando hasta obtener una respuesta satisfactoria. A continuación, se ajusta la ganancia integral (Ki) del controlador. Si el sistema no está respondiendo de manera adecuada, se aumenta o disminuye el valor de Ki hasta obtener una respuesta satisfactoria. El proceso se repite para la ganancia derivativa (Kd), si es necesario.

El ajuste manual es un método laborioso y requiere de mucha experiencia y conocimientos técnicos. Aunque puede proporcionar un rendimiento óptimo en algunos casos, se recomienda utilizar los métodos automatizados de ajuste de controladores PID cuando sea posible, para garantizar la precisión y eficiencia del proceso.

`````python
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

#Se crean las funciones de tipo callback para obtener los datos de los tópicos y nodos para su posterior uso
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
        #Se obtiene un tiempo actual con la función de rospy.get_time()
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

`````

### Set_point.py

Para el archivo de "Set_point" programado en python recibimos mensajes customizados y parámetros tanto para generar la señal como para publicarla, dichos parámetros son la frecuencia de la senoidal y la amplitud. Es así que la señal senoidal será enviada al controlador para que este próximamente pueda enviarlo al sistema y ejecutar las funciones correspondientes.


`````python
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

`````


### motor_control.launch

`````
<?xml version="1.0" ?>
<launch>
    <rosparam file = "$(find pid_control)/config/system_params.yaml" command = "load"  />
    <rosparam file = "$(find pid_control)/config/setpoint_params.yaml" command = "load"  />
    <rosparam file = "$(find pid_control)/config/control_params.yaml" command = "load"  />

    <node name="Set_Point_Generator" pkg="pid_control" type="set_point_generator.py" output="screen"/>
    <node name="Controller" pkg="pid_control" type="controller.py" output="screen"/>
    <node name="Motor_Sim" pkg="pid_control" type="system.py" output="screen"/>
    <node name="rqt_plot" pkg="rqt_plot" type="rqt_plot" output="screen"  args="/motor_output/output /set_point/entrada "/>
    <node name="rqt_graph" pkg="rqt_graph" type="rqt_graph" output="screen"/>
</launch>
`````

## Resultados

**<p align="center"> Representación de cada uno de los nodos y tópicos utilizando el comando rqt_graph</p>**
<p align="center">
  <img src="https://github.com/engelSprt/Retos_Manchester_Robotics/blob/main/Challenge%202/images/Graph_controller2.png" />
</p>
Podemos observar que la conexión entre tópicos y nodos es la correcta.

**<p align="center"> Graficación de la respuesta del sistema con el controlador junto al set_point utilizando el comando de rqt_plot</p>**
<p align="center">
  <img src="https://github.com/engelSprt/Retos_Manchester_Robotics/blob/main/Challenge%202/images/Respuesta_controllador2.png" />
</p>
Se concluye observando esta imágen que el controlador cumple su función al corregir la salida del sistema para minimizar las oscilaciones y el ruido presentado por el sistema.

**<p align="center"> Datos mostrados en la consola para el error</p>**
<p align="center">
  <img src="https://github.com/engelSprt/Retos_Manchester_Robotics/blob/main/Challenge%202/images/Consola2.png" />
</p>
Podemos observar un error bastante aceptable, por lo que se puede decir que el controlador está implementado de una forma correcta.

## Conclusiones

Este segundo reto sin duda fue más desafiante y un gran salto de complejidad en comparación del reto anterior, sin embargo, pudimos aplicar el material de autoestudio
a la solución de una situación problema de esta naturaleza, el cual fue desarrollar un controlador PID para un sistema de primer orden y graficar los resultados. 
Las cuatro actividades de autoestudio de esta semana fueron de gran ayuda para la resolución del reto, ya que nos permitió entender el uso de ROS namespaces, 
ROS parameter files y ROS custom messages al aplicarlos a una actividad que previamente habíamos realizado. La única mejora que consideramos es haber tenido más 
conocimientos y prácticas previas sobre diseñar controladores PID, debido a que si fue un gram desafío diseñarlo en python sin tener mucho conocimiento acerca de ello.
Por otro lado, las nuevas herramientas de ROS que empleamos en este reto, fue una gran aportación al curso y a  nuestros aprendizajes de la unidad de formación.

