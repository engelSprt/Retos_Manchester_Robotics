# CHALLENGE 2

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

Esta semana se estudia el rosserial, una herramienta con el propósito de estandarizar la comunicación entre las computadoras y el hardware robótico.
Rosserial permite seguir practicando con "topics", "services" y "loggin features" de ROS.

Para lograr el objetivo de esta práctica se investigará acerca la señal PWM y como se aplica para lograr regular un motor con encoder.

Esta páctica de ROS que aplica los conceptos de las sesiones anteriores requiere usar el IDE de arduino, por lo que andemás de descargarlo en Ubuntu e 
instalar las librería de ROS, recordaremos la codificación de este lenguaje de programación y su utilidad con ROS. 

## Objetivos
Este desafío está destinado a entender el PWM en un motor DC con ROS y arduino.

• La actividad consiste en crear nodos para regular la
velocidad del motor.

• El motor se controlará mediante un ordenador externo, un
microcontrolador (arduino) y un controlador de motor (puente h - motor driver).

• El nodo “/motor” debe ejecutarse en el microcontrolador.

• La salida del Arduino es la señal PWM y una dirección al controlador de motor.

• El controlador del motor genera la potencia necesaria para el motor DC.

## Introducción
La robótica es la rama de la ingeniería mecánica, electrónica e informática que se especializa en el diseño y construcción de robots que realizan trabajos, generalmente en la sustitución de la mano de obra humana.

La evolución de la robótica se ha visto desarrollada en los últimos años, sobre todo con la automatización y la industria 4.0, debido a que genera beneficios económicos significativos, por lo que manejar la herramienta R.O.S. es una necesidad de nuestra carrera profesional.

Continuando con el aprendizaje de las herramientas de ROS tenemos los siguientes elementos importantes:
<p align="center">
  <img src=" " />
</p>

## Solución del problema
Para la solución de este reto se cuentan con un archivo codificado en lenguaje arduino, llamado: .ino. Además del 
archivo.launch para ejecutar los nodos. 
A continuación se describe la funcionalidad:

### .ino

`````c

`````




### motor_control.launch

`````

`````

## Resultados

**<p align="center"> Ejecución de ROS en el ordenador</p>**

**<p align="center"> Video de demosración</p>**

**<p align="center"> Datos mostrados en la consola</p>**

## Conclusiones


