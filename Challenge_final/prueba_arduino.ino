//NODO de ROS en Arduino

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h> //Incluir las librerias para los mensajers customizados
#include <std_msgs/Int16.h>
#include <motor_input.h>
#include <motor_output.h>
#define MOTOR 0
#define CW    1 //Clock wise
#define CCW   2 //Counter clock wise
#define STOP 0
//Pins entre arduino y puente h del diagrama de conexiones de MCR2
#define PIN1_MOTOR #
#define PIN2_MOTOR #
#define PWM #
//pins para arduino
const int Pin1_encoder = 2;
const int Pin2_encoder = 3;
//valor del registro puede cambiar a través de acciones fuera del programa
volatile int64_t position = 0;

//Los mensajes del encoder se envian a traves de un mensaje customizado para que el nodo dentro de la computadora pueda calcular y enviar cada uno
//De los parametros de velocidad angular.

//El mensaje de la respuesta PID publica a la entrada del motor mediante un mensaje customizado

//lib ros
ros::NodeHandle  nh;

//define el tipo de datos del topic.
custom_msg::motor_output sensor_encoder;

//publicador
ros::Publisher Value_position("Value_position", &sensor_encoder);

//suscriptor para el tema /motor_input.
ros::Subscriber<custom_msg::motor_input> pwm_motor("motor_input", &signal_pwm);

//funcion para prender el motor con los parámtros:  motor(0 ó 1), direccion cw ó ccw ,  pwm (0 - 255)
void motor_on(int16_t motor, int16_t direction, Float32 pwm){

  if(motor == MOTOR){

    if(direction == CW){
      digitalWrite(PIN1_MOTOR, LOW);
      digitalWrite(PIN2_MOTOR, HIGH);
    }
    else if(direction == CCW){
      digitalWrite(PIN1_MOTOR, HIGH);
      digitalWrite(PIN2_MOTOR, LOW);
    }
    else{
      digitalWrite(PIN1_MOTOR, LOW);
      digitalWrite(PIN2_MOTOR, LOW);
    }

   analogWrite(PWM, pwm);
  }

}
//funciones  para leer los valores del encoder
//funciones de rutina de servicio de interrupción para actualizar el valor del encoder sobre el movimiento del eje del motor
void Encoder_out1(){
  if (digitalRead(Pin1_encoder) != digitalRead(Pin2_encoder)){
    position++;
  }
  else{
    position--;
  }
}
void Encoder_out2(){
  if (digitalRead(Pin1_encoder) == digitalRead(Pin2_encoder)){
    position++;
  }
  else{
    position--;
  }
}


// función para mover el motor de acuerdo con el signo del valor PWM que se le da
void signal_pwm( const custom_msg::motor_input &value_pwm){
  Float32 pwm =0;
  pwm = value_pwm.input;

  if ( pwm > 0 ){
  motor_on(MOTOR,CCW,pwm);
  }
  else{
  motor_on(MOTOR,CW,abs(pwm));
  }

}

//setupp de configuración de los pines del Encoder como pines de interrupción.
void setup(){
  //lib ros
  nh.initNode();
  nh.advertise(Value_position);
  nh.subscribe(pwm_motor);

  //Se le agrega al pin una resistencia de pull-up interna y se configura como entrada a los pines de interrupcion.
  pinMode (Pin1_encoder, INPUT_PULLUP);
  pinMode (Pin2_encoder, INPUT_PULLUP);

  //registro de temporizador para modificar velocidades del PWM ATMEGA328 - arduino UNO
  //timers para interrupción

  //timer1 - PWM frequencia = 490.20 Hz en pines:  D9 & D10
  //TCCR1B = TCCR1B & B11111000 | B00000011;

  //timer2 - PWM frequencia = 490.20 Hz en pines:  D3 & D11 para que sea más raṕida la interrupcion
  TCCR2B = TCCR2B & B11111000 | B00000100;

  //las interrupciones por "cambio" que nos recomendó el profe
  attachInterrupt (digitalPinToInterrupt (Pin1_encoder), Encoder_out1, CHANGE);
  attachInterrupt (digitalPinToInterrupt (Pin2_encoder), Encoder_out2, CHANGE);
}

//loop para publicar continuamente los valores del Encoder actualizados en el topic definido.
void loop(){
  //lib ros
  nh.loginfo("Encoder data");
  sensor_encoder.output = position;
  Value_position.publish( &sensor_encoder );
  nh.spinOnce();
  delay(100);

}
