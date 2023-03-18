
#include <ros.h>
#include <std_msgs/Float32.h>
#include <digitalWriteFast.h>
#define pi 3.1416

//Declaracion de los pines
const int EncA = 2;
const int EncB = 3;
const int In1 = 4; // Digital output pin
const int In2 = 5; // Digital output pin
const int EnA = 6; // PWM output pin
float conversion, posicion, posactual = 0, posanterior = 0, velocidad = 0;
float resolucion = 0.0306;  //Definir resolución del encoder
int pulsos = 11789;//11789; Numero de pulsos a la salida del motorreductor

String dir = "CW";
String last_dir = "CCW";
long current_time = 0;
long last_time = 0;
volatile bool lecA = 0;
volatile bool lecB = 0;

long contador = 0, contaux = 0;
float dt = 0.02;

void Encoder(){
  lecA = digitalReadFast(EncA);
  lecB = digitalReadFast(EncB);

  if(lecA == lecB){
    contador ++;
    contaux++;
    posicion = contador * resolucion; //Convertir a grados
    posactual = contaux * resolucion;
    if(contador >= pulsos)
    { 
      contador = 0;
    }
  }
  else
  {
  contador --;
  contaux--;
  posicion = contador * resolucion;
  posactual = contaux * resolucion; //Convertir a grados
  if(contador <= -pulsos)
  {
    contador = 0;
  }
 }
}

void pwm_int(const std_msgs::Float32& msg) {

  float val_received = msg.data;

  if (val_received > 0) {
    dir = "CW";
  }
  else {
    dir = "CCW";
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
std_msgs::Float32 velocidad_out;
//ros::Subscriber<std_msgs::Float32> motor("/motor_input", pwm_int);
ros::Subscriber<std_msgs::Float32> motor("/motor_input", pwm_int);
ros::Publisher motor_output("/motor_output", &velocidad_out); 
void setup() {
  nh.initNode();
  nh.subscribe(motor);
  nh.advertise(motor_output);

  pinMode(EnA, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);

  pinMode(EncA, INPUT_PULLUP);
  pinMode(EncB, INPUT_PULLUP);

  //attachInterrupt (digitalPinToInterrupt (EncA), leerA, CHANGE);
  attachInterrupt (digitalPinToInterrupt (EncA), Encoder, CHANGE);   // <------------- 
}

void loop() {
  
  current_time = millis();
  // Period of time "interval"
  if (current_time - last_time > dt) {
    last_time = current_time;
    //Método de Euler para la estimación de la velocidad
    velocidad = ((posactual - posanterior) / 0.015); //Tiempo de muestreo = 0.015 segundos
    velocidad = velocidad * pi / 180;
    //Se pregunta por la velocidad, cuando hay una inversion de giro, para hacerla positiva
    //if (velocidad < 0){
      //velocidad = velocidad;
    //Actualizar la lectura de la posición anterior
      posanterior = posactual;
    //}
  }
  velocidad_out.data = velocidad*-1;
  //nh.loginfo(velocidad_out);
  motor_output.publish(&velocidad_out); 
  nh.spinOnce();
  
/*
    current_time = millis();
  // Period of time "interval"
  //if (current_time - last_time > dt) {
    last_time = current_time;
    //Método de Euler para la estimación de la velocidad
    velocidad = ((posactual - posanterior) / 0.015); //Tiempo de muestreo = 0.015 segundos
    velocidad = velocidad * pi / 180;
    //Se pregunta por la velocidad, cuando hay una inversion de giro, para hacerla positiva
    //if (velocidad < 0){
      //velocidad = velocidad;
    //Actualizar la lectura de la posición anterior
    posanterior = posactual;
    //}
  //}
    velocidad_out.data = velocidad;
  //nh.loginfo(velocidad_out);
    motor_output.publish(&velocidad_out); 
    nh.spinOnce();
    */
    
}
