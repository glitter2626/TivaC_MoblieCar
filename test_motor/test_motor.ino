#include <Motordriver.h>

#define TRIGGER_PIN         12
#define ECHO_PIN            13
#define IR_PIN              26   
#define MOTOR_LEFT_A        11
#define MOTOR_LEFT_B        31
#define MOTOR_LEFT_PWM      14
#define MOTOR_RIGHT_A       32
#define MOTOR_RIGHT_B       33
#define MOTOR_RIGHT_PWM     15
#define ENCODER_LEFT_A      71
#define ENCODER_LEFT_B      72
#define ENCODER_RIGHT_A     73
#define ENCODER_RIGHT_B     74

Motordriver motor(MOTOR_LEFT_A, MOTOR_LEFT_B, MOTOR_LEFT_PWM, MOTOR_RIGHT_A, MOTOR_RIGHT_B, MOTOR_RIGHT_PWM);
void setup() {
  // put your setup code here, to run once:
  motor.setup();
}

void loop() {
  // put your main code here, to run repeatedly: 
  motor.drive(50,50);
  //delay(1000);
}
