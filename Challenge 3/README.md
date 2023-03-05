# CHALLENGE 3

<p align="center">
  <img src="https://github.com/engelSprt/Retos_Manchester_Robotics/blob/main/Challenge%201/Imagenes/tecnologico-de-monterrey-blue.png" />
</p>


**<p align="center">Instituto Tecnológico y de Estudios Superiores de Monterrey</p>**
<p align="center">TE3001B.101.</p>
<p align="center">Fundamentación de robótica (Gpo 101)</p>
<p align="center">Semestre: febrero - junio 2023.</p>
<p align="center">Challenge 3</p>
<p align="center"> Integrantes:</p>
<p align="center">Fredy Yahir Canseco Santos		A01735589</p>
<p align="center">José Ángel Ramírez Ramírez		A01735529</p>
<p align="center">Daniel Ruán Aguilar			A01731921</p>
<p align="center">Fecha de entrega: 05 de marzo del 2023</p>


## Resumen

Esta semana se estudia rosserial, una herramienta con el propósito de estandarizar la comunicación entre las computadoras y el hardware robótico.
Rosserial permite seguir practicando con "topics", "services" y "loggin features" de ROS.

Para lograr el objetivo de esta práctica se investigará acerca la señal PWM y como se aplica para lograr regular un motor con encoder.

Esta páctica de ROS que aplica los conceptos de las sesiones anteriores requiere usar el IDE de arduino, por lo que andemás de descargarlo en Ubuntu e 
instalar las librería de ROS, investigaremos la codificación de este lenguaje de programación y su utilidad con ROS. 

## Objetivos

Este desafío está destinado a entender el envió de una señal PWM en un motor DC utilizando ROS y arduino.

• La actividad consiste en crear nodos para regular la
velocidad del motor.

• El motor se controlará mediante un ordenador externo, un
microcontrolador (arduino) y un controlador de motor (puente h - motor driver).

• El nodo “/motor” debe ejecutarse en el microcontrolador.

• La salida del Arduino es la señal PWM y una dirección al controlador de motor.

• El controlador del motor genera la potencia necesaria para el motor DC.

## Introducción

<p align="center">
  <img src=" " />
</p>

## Solución del problema

Para la solución de este reto se cuenta con un archivo codificado en lenguaje arduino, llamado: .ino. Además del archivo.launch para la ejecuación. 

A continuación se describe la funcionalidad:

### .ino

`````c

`````



### motor_control.launch

`````

`````

## Resultados  

**<p align="center"> Ejecución de ROS en el ordenador</p>**

<p align="center">
  <img src=" " />
</p>

<p align="center">
  <img src=" " />
</p>

Se envía el parametro de PWM (0-255) a través de la terminar mientras se ejecuta roscore.

**<p align="center"> Video de demostración</p>**


En este video se puede ver como cambia la velocidad del motor a través del PWM que se envía desde el ordenador.


## Conclusiones

Este reto fue bastante interesante por la manera en que se comunica un ordenador con ROS y un hardware externo. Además de que será muy útil para darle 
continuidad a las siguientes etapas del curso y unir todas las prácticas en el reto final planeado por MCR2. Consideramos que pudimos lograr el propósito 
de este mini challenge. Para enviar una simple señal de PWM desde arduino tuvimos que recordar la manera de programar en este IDE, lo cual es muy similar 
a programar en C. En adición investgamos la manera en que se usaba la librería de ROS para la comunicación, en especial entendimos como 
funciona publisher y subscriber en este entorno de programación.
