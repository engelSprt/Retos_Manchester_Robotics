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

Para la solución de este reto se cuenta con un archivo codificado en lenguaje arduino, llamado: PWM.ino, El cual es nuestro nodo "/motor" encargado de recibir un valor entre -1 y 1, y traducirlo en el pwm y direccion, para posteriormente enviar esta informacion a nuestro motor.

### PWM.ino

Antes de nuestro setup, declaramos los pines a utlizar para  controlar el giro de nuestro motor y activar el puente H. Asi mismo creamos dos variables que almacenaran tanto la direccion actual como la direccion anterior, para de esta manera saber cuando hubo un cambio de giro. Por creamos una funcion que sera el callback de nuestro suscriptor, en esta funcion se recibe el valor que se encia desde la computadora externa, primero se verifica si hubo un cambio de giro, si esto ocurre se apaga por un pequeño momento el motor para no dañar nuestro puente H, despues se mapea nuestro valor recibido para enviarlo al motor.


`````c

#include <ros.h>
#include <std_msgs/Float32.h>

                                            //pines a utlizar
const int In1 = 2; // Analog output pin 
const int In2 = 3; // Analog output pin 
const int EnA = 9; // Activar o desactivar Puente H

                                            //variables de direccion actual y direccion anterior

char dir, last_dir='r';
                                            //Funcion que publica el pwm y la direccion                                            
void pwm_int(const std_msgs::Float32 msg) {

  float val_recived = msg.data;
  
                                            //verificamos si hubo un cambio de direccion
  if(val_recived > 0){
    dir = 'r';
    }
  else{
    dir = 'l';
    }
                                            //Si hubo cambio de direccion apagamos por 100ms el motor
  if(dir =! last_dir){
    digitalWrite(In1, LOW);
    digitalWrite(In2, LOW);
    delay(100);
    }

  last_dir = dir;

  
                                              //mapeamos el valor recibido 
  
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

                                            //creamos nuestro suscriptor al topico "cmd_pwm"

ros::NodeHandle  nh;

std_msgs::Float32 pwm_value;

ros::Subscriber<std_msgs::Float32> motor("cmd_pwm", pwm_int);  

`````
Se inicializa nuestro nodo y nuestro suscriptor, y declaramos nuestros pines como salidas

`````c

void setup()
{

                                            //iniciamos nuesto nodo y activamos nuestro pines como salida
  nh.initNode();
  nh.subscribe(motor);

  pinMode(EnA, OUTPUT); 
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT); 



}

`````

`````c

void loop()
{


    
  nh.spinOnce();
  delay(100);
}


`````

### pwm_input.py
Como extension a este reto, se agrego un nodo en python, el cual contiene 4 funciones que representan 4 señales, senoidal, escalon unitario, cuadrada y diente de sierra, cualquiera de estas señales puede ser enviada al topico "cmd_input", mismo al cual esta suscrito el nodo del arduino y del cual recibe el valor entre -1 y 1 para posteriormente convertirlo en direccion y pwm. La señal que se dese enviar al topico mencionado anteriormente, debe ser elegida mediante la linea de comandos con _rosparam set_ 
`````python

#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32, String


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
