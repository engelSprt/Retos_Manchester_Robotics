#include <ros.h>
#include <std_msgs/Float32.h>

const int In1 = 2; // Digital output pin 
const int In2 = 3; // Digital output pin 
const int EnA = 9; // PWM output pin

char dir = 'r', last_dir = 'r';

void pwm_int(const std_msgs::Float32& msg) {
  
  float val_received = msg.data;
  
  if (val_received > 0) {
    dir = 'r';
  }
  else {
    dir = 'l';
  }

  if (dir != last_dir) {
    digitalWrite(In1, LOW);
    digitalWrite(In2, LOW);
    delay(100);
  }

  last_dir = dir;
 
  int pwm = int(val_received*255);
  //int pwm = map(val_received, -1, 1, -255, 255);
  if (pwm >= 0 && pwm <= 255) {
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
    analogWrite(EnA, pwm);
  }
  else if (pwm >= -255 && pwm <= 0) {
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
    analogWrite(EnA, -pwm);
  }
  
}

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Float32> motor("cmd_pwm", pwm_int);

void setup() {
  nh.initNode();
  nh.subscribe(motor);

  pinMode(EnA, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
}

void loop() {
  nh.spinOnce();
}
