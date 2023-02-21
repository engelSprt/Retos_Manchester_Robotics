# CHALLENGE 1

<p align="center">
  <img src="https://github.com/engelSprt/Retos_Manchester_Robotics/blob/main/Challenge%201/Imagenes/tecnologico-de-monterrey-blue.png" />
</p>


**<p align="center">Instituto Tecnológico y de Estudios Superiores de Monterrey</p>**
<p align="center">TE3001B.101.</p>
<p align="center">Fundamentación de robótica (Gpo 101)</p>
<p align="center">Semestre: febrero - junio 2023.</p>
<p align="center">Challenge 1</p>
<p align="center"> Integrantes:</p>
<p align="center">Fredy Yahir Canseco Santos		A01735589</p>
<p align="center">José Ángel Ramírez Ramírez		A01735529</p>
<p align="center">Daniel Ruán Aguilar			A01731921</p>
<p align="center">Fecha de entrega: 20 de febrero del 2023</p>


## Resumen

En el reto de esta primera semana, ponemos en práctica los conceptos aprendidos sobre la tarea más sencilla que se puede realizar en ROS, la comunicación entre dos nodos. 
En esta práctica específicamente se aplican conceptos como:
* Creación de nodos
* Comunicación entre nodos
* Topics
* Mensajes


## Objetivos
El primer nodo que se crea actuará como un simple generador de señales, en este caso generará una señal sinusoidal. 
El segundo nodo tendrá la tarea de “procesar”, tomará la señal generada por el nodo anterior y la modificará, para generar una señal procesada.
Ambas señales se visualizarán utilizando rqt_plot, mientras que las diferentes terminales mostrarán la información de las señales generadas 
Finalmente, se generará un archivo de lanzamiento (launch file) para ejecutar ambos nodos, terminales y rqt_plot al mismo tiempo.

## Introducción
## Solución del problema
## Resultados
<p align="center">
  <img src="https://github.com/engelSprt/Retos_Manchester_Robotics/blob/main/Challenge%201/Imagenes/senoidal.png" />
</p>

<p align="center">
  <img src="https://github.com/engelSprt/Retos_Manchester_Robotics/blob/main/Challenge%201/Imagenes/cambio_fase.png" />
</p>

<p align="center">
  <img src="https://github.com/engelSprt/Retos_Manchester_Robotics/blob/main/Challenge%201/Imagenes/graph.png" />
</p>

<p align="center">
  <img src="https://github.com/engelSprt/Retos_Manchester_Robotics/blob/main/Challenge%201/Imagenes/challenge1.png" />
</p>

`````python
import rospy
import numpy as np
from std_msgs.msg import Float32

if __name__=='__main__':

    #Finish configuring your node here (part of the code has already been written as a hint)
    rospy.init_node("signal_generator")
    rate = rospy.Rate(10)
    signal_pub = rospy.Publisher("signal",Float32, queue_size=10)
    time_pub = rospy.Publisher("time",Float32, queue_size=10)

    while not rospy.is_shutdown():
            time = rospy.get_time()
            funcion = np.sin(time)
            rospy.loginfo(funcion)
            signal_pub.publish(funcion)
            time_pub.publish(time)

            rate.sleep()

`````

`````python
#Aqui va el codigo
`````


`````python
#Aqui va el codigo
`````

## Concluciones



  









              	

		




