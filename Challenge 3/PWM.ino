
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
