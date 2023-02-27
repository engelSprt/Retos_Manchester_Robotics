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

`````python

`````

### Controller.py

`````python

`````

### Set_point.py

`````python

`````


### motor_control.launch

`````

`````

## Resultados

**<p align="center"> Representación de cada uno de los nodos y tópicos utilizando el comando rqt_graph</p>**

**<p align="center"> Graficación de la respuesta del sistema con el controlador junto al set_point utilizando el comando de rqt_plot</p>**

**<p align="center"> Datos mostrados en la consola</p>**

## Conclusiones

Este segundo reto sin duda fue más desafiante y un gran salto de complejidad en comparación del reto anterior, sin embargo, pudimos aplicar el material de autoestudio
a la solución de una situación problema de esta naturaleza, el cual fue desarrollar un controlador PID para un sistema de primer orden y graficar los resultados. 
Las cuatro actividades de autoestudio de esta semana fueron de gran ayuda para la resolución del reto, ya que nos permitió entender el uso de ROS namespaces, 
ROS parameter files y ROS custom messages al aplicarlos a una actividad que previamente habíamos realizado. La única mejora que consideramos es haber tenido más 
conocimientos y prácticas previas sobre diseñar controladores PID, debido a que si fue un gram desafío diseñarlo en python sin tener mucho conocimiento acerca de ello.
Por otro lado, las nuevas herramientas de ROS que empleamos en este reto, fue una gran aportación al curso y a  nuestros aprendizajes de la unidad de formación.

