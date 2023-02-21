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
Crear dos nodos que se comuniquen a traves de 2 topicos para compartir informacion.  
  
El primer nodo que se crea actuará como un simple generador de señales, en este caso generará una señal sinusoidal.   
  
El segundo nodo tendrá la tarea de “procesar”, tomará la señal generada por el nodo anterior y la modificará, para generar una señal procesada.  
  
Ambas señales se visualizarán utilizando rqt_plot, mientras que las diferentes terminales mostrarán la información de las señales generadas   
  
Finalmente, se generará un archivo de lanzamiento (launch file) para ejecutar ambos nodos, terminales y rqt_plot al mismo tiempo.  
  

## Introducción
La robótica es la rama de la ingeniería mecánica, electrónica e informática que se especializa en el diseño y construcción de robots que realizan trabajos, generalmente en la sustitución de la mano de obra humana.
La evolución de la robótica se ha visto desarrollada en los últimos años, sobre todo con la automatización y la industria 4.0, debido a que genera beneficios económicos significativos.
Es por esto que el aprendizaje con la herramienta ROS es fundamental para nuestra carrera.

## Solución del problema
Para la solución de este reto se cuentan con dos archivos codificados en lenguaje Python, llamados: signal_generator y process. Se comenzará por describir la funcionalidad de cada uno de ellos en el siguiente apartado:

### Signal Generator.py
En el archivo Signal generator se comienza por agregar cada una de las bibliotecas y dependencias correspondientes que permitirán a los nodos intercambiar mensajes entre sí, y además ejecutar las tareas correspondientes. Posteriormente se inicializa el nodo y se especifica que trabajará con una frecuencia de 10 Hz, enseguida de esto se declaran dos variables para publicar datos, variables las cuales corresponden a los datos de la señal y del tiempo, es decir, este nodo que transmite datos publicará el nombre de los tópicos (/signal, /time), el tipo de mensaje a enviar y los datos como tal. Posteriormente en el ciclo while, mientras ros no se le ordene apagarse realizaremos lo siguiente: la variable time será utilizada para el tiempo del cual depende nuestra función senoidal, después la variable función servirá generar la señal senoidal enviada por el nodo, dicha señal dependerá del tiempo anteriormente mencionado, enseguida de esto, con la función rospy.loginfo se imprimirán los valores de la señal senoidal dentro de la consola para poder visualizarlos de mejor manera, y, finalmente publicamos la señal senoidal junto con el tiempo.

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
### Process.py

`````python
import rospy
import numpy as np
from std_msgs.msg import Float32

#Global variable to store the data from the message in the /signal topic
signal_data = 0
time_data = 0

# Example Callback Function (Hint)
def callback(msg):
    global signal_data
    signal_data = msg.data


def callback_time(msg):
    global time_data
    time_data = msg.data


if __name__=='__main__':

    #Finish configuring your node here (part of the code has already been written as a hint)
    rospy.init_node("process")
    #Subscriber example (Hint)
    rospy.Subscriber("signal", Float32, callback)
    rate = rospy.Rate(10)
    rospy.Subscriber("time", Float32, callback_time)
    rate = rospy.Rate(10)
    nueva_pub = rospy.Publisher("proc_signal",Float32, queue_size=10)
    #rate = rospy.Rate(10)


    while not rospy.is_shutdown():
        #t = (time_data)*(np.pi*2)
        t = time_data
        #half_signal = signal_data + 3
        #half_signal = half_signal * 0.5
        #t0=np.pi*2
        #t0=np.pi*2
        half_signal = ((signal_data * np.cos(10)) + np.cos(t) * np.sin(20)) * -np.cos(t)
        #half_signal = (np.sin(t) * np.cos(5)) + (np.cos(t) * np.sin(5))
        #signal_data = (np.sin(t) * np.cos(t0)) + (np.cos(t) * np.sin(t0))
        half_signal = half_signal + 3
        half_signal = half_signal * 0.5
        #half_signal = (half_signal * np.exp(-1j*t0))
        rospy.loginfo(half_signal)
        nueva_pub.publish(half_signal)

        rate.sleep()
`````


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


## Conclusiones

Consideramos que esta primera práctica nos ayudó a terminar de comprender cómo funcionan las herramientas básicas de ROS, las cuales nos serán útiles para desarrollar los futuros retos del curso. Pudimos entender y aplicar los conceptos aprendidos en la primera semana para resolver un reto planteado por el socio formador, que en este caso fue la comunicación entre dos nodos para modificar una función sin(x) y desplegar los resultados.



  









           
