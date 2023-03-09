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
  <img src="https://github.com/engelSprt/Retos_Manchester_Robotics/blob/main/Challenge%203/images/Mapa%20Mental%20Ideas%20Corporativo%20rosa%20y%20Naranja.png" />
</p>

## Solución del problema

Para la solución de este reto se cuenta con un archivo codificado en lenguaje arduino, llamado: .ino.

A continuación se describe la funcionalidad:

### .ino

`````c

#include <ros.h>
#include <std_msgs/Float32.h>

// These constants won't change. They're used to give names to the pins used:
//const int variador = A0;  // Analog input pin that the potentiometer is attached to
const int In1 = 2; // Analog output pin 
const int In2 = 3; // Analog output pin 
const int EnA = 9; // Activar o desactivar Puente H

char dir, last_dir='r';

void pwm_int(const std_msgs::Float32 msg) {

  float val_recived = msg.data;
  

  if(val_recived > 0){
    dir = 'r';
    }
  else{
    dir = 'l';
    }

  if(dir =! last_dir){
    digitalWrite(In1, LOW);
    digitalWrite(In2, LOW);
    delay(100);
    }

  last_dir = dir;

  
  
  
  int pwm = val_recived*255;
  
  if(val_recived>=0 && val_recived <= 1){

    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
    analogWrite(EnA, pwm);
  }
  else if(val_recived >= -1 && val_recived <= 0){
    
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
    analogWrite(EnA, -pwm);
  }
  
}



ros::NodeHandle  nh;

std_msgs::Float32 pwm_value;

ros::Subscriber<std_msgs::Float32> motor("cmd_pwm", pwm_int);  



void setup()
{

  nh.initNode();
  nh.subscribe(motor);

  pinMode(EnA, OUTPUT); 
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT); 



}

void loop()
{


    
  nh.spinOnce();
  delay(100);
}


`````

## Resultados  

**<p align="center"> Video de demostración con la explicación de la ejecución</p>**

https://drive.google.com/file/d/1HZ1CPLM-PxY7eF_v2G-9Eb3xtx01ICvY/view?usp=sharing

En este video se puede ver como cambia la velocidad del motor a través del PWM que se envía desde el ordenador, se envía el parametro de PWM a través de 
la terminar mientras se ejecuta roscore..


## Conclusiones

Este reto fue bastante interesante por la manera en que se comunica un ordenador con ROS y un hardware externo. Además de que será muy útil para darle 
continuidad a las siguientes etapas del curso y unir todas las prácticas en el reto final planeado por MCR2. Consideramos que pudimos lograr el propósito 
de este mini challenge. Para enviar una simple señal de PWM desde arduino tuvimos que recordar la manera de programar en este IDE, lo cual es muy similar 
a programar en C. En adición investgamos la manera en que se usaba la librería de ROS para la comunicación, en especial entendimos como 
funciona publisher y subscriber en este entorno de programación.
