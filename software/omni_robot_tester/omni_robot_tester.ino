#include "robomodule_lib.h"
#include "commu_data_exchange.h"
#include "due_can.h"

#define DEBUG_MODE


const int MOTOR1_DEGREE = 0+90;
const int MOTOR2_DEGREE = 120+90;
const int MOTOR3_DEGREE = 240+90;


const int CANBUS_ENABLE_PIN = 23;


CAN_FRAME outgoing_frame;
Speed_cmd mega_serialFrame;

HardwareSerial& mega_serial = Serial3;

int max_speed = 300;

//Variable


void setup() {
  //Can b
  #ifdef DEBUG_MODE
    Serial.begin(115200);
    Serial.print("Start\n");
  #endif
  mega_serial.begin(115200);
  Can0.begin(CAN_BPS_1000K); //Setup CAN bus

  pinMode(CANBUS_ENABLE_PIN, OUTPUT);
  digitalWrite(CANBUS_ENABLE_PIN, LOW);


  
  //CAN buss setip
  module_reset(outgoing_frame, 0x0); //Reset all connected module 
  Can0.sendFrame(outgoing_frame);
  delay(500);
  
  module_setMode(outgoing_frame, 0x0, SPEED_MODE); //Set Mode
  Can0.sendFrame(outgoing_frame);
  delay(500);

  set_Max_PWM(5000);
}

void loop() {

#ifdef DEBUG_MODE
    int pwm_speed = 5000;

    int motor1_speed = 0;
    int motor2_speed = 0;
    int motor3_speed = 0;

    int remap_vx = 0;
    int remap_vy = 0;
    int remap_wr = 0;
    if(Serial.available()){
      switch(Serial.read()){
        case 'c'://Stop
        Serial.print("Stop\n");
          remap_vx = 0;
          remap_vy = 0;
          remap_wr = 0;
          
          //Compute calculation
          motor1_speed = (-sin(to_rad(MOTOR1_DEGREE))*remap_vx)+(cos(to_rad(MOTOR1_DEGREE))*remap_vy)+(1*remap_wr);
          motor2_speed = (-sin(to_rad(MOTOR2_DEGREE))*remap_vx)+(cos(to_rad(MOTOR2_DEGREE))*remap_vy)+(1*remap_wr);
          motor3_speed = (-sin(to_rad(MOTOR3_DEGREE))*remap_vx)+(cos(to_rad(MOTOR3_DEGREE))*remap_vy)+(1*remap_wr);

          Serial.print("Motor1_speed: ");Serial.print(motor1_speed);Serial.print("\n");
          Serial.print("Motor2_speed: ");Serial.print(motor2_speed);Serial.print("\n");
          Serial.print("Motor3_speed: ");Serial.print(motor3_speed);Serial.print("\n");
      
          //Set motor 1
          //module_setSpeed(outgoing_frame, 0x01, motor1_speed);
          outgoing_frame.id = 0x014;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor1_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor1_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
          //Set motor 2
          //module_setSpeed(outgoing_frame, 0x02, motor2_speed);
          outgoing_frame.id =  0x024;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor2_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor2_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
          //Set motor 3
          //module_setSpeed(outgoing_frame, 0x03, motor3_speed);
          outgoing_frame.id =  0x034;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor3_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor3_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
        break;

        case 'w'://Up
        Serial.print("Up\n");
          remap_vx = 0;
          remap_vy = max_speed;
          remap_wr = 0;
          
          //Compute calculation
          motor1_speed = (-sin(to_rad(MOTOR1_DEGREE))*remap_vx)+(cos(to_rad(MOTOR1_DEGREE))*remap_vy)+(1*remap_wr);
          motor2_speed = (-sin(to_rad(MOTOR2_DEGREE))*remap_vx)+(cos(to_rad(MOTOR2_DEGREE))*remap_vy)+(1*remap_wr);
          motor3_speed = (-sin(to_rad(MOTOR3_DEGREE))*remap_vx)+(cos(to_rad(MOTOR3_DEGREE))*remap_vy)+(1*remap_wr);

          Serial.print("Motor1_speed: ");Serial.print(motor1_speed);Serial.print("\n");
          Serial.print("Motor2_speed: ");Serial.print(motor2_speed);Serial.print("\n");
          Serial.print("Motor3_speed: ");Serial.print(motor3_speed);Serial.print("\n");
      
          //Set motor 1
          //module_setSpeed(outgoing_frame, 0x01, motor1_speed);
          outgoing_frame.id = 0x014;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor1_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor1_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
          //Set motor 2
          //module_setSpeed(outgoing_frame, 0x02, motor2_speed);
          outgoing_frame.id =  0x024;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor2_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor2_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
          //Set motor 3
          //module_setSpeed(outgoing_frame, 0x03, motor3_speed);
          outgoing_frame.id =  0x034;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor3_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor3_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
        break;

        case 's'://Down
        Serial.print("Down\n");
          remap_vx = 0;
          remap_vy = -max_speed;
          remap_wr = 0;
          
          //Compute calculation
          motor1_speed = (-sin(to_rad(MOTOR1_DEGREE))*remap_vx)+(cos(to_rad(MOTOR1_DEGREE))*remap_vy)+(1*remap_wr);
          motor2_speed = (-sin(to_rad(MOTOR2_DEGREE))*remap_vx)+(cos(to_rad(MOTOR2_DEGREE))*remap_vy)+(1*remap_wr);
          motor3_speed = (-sin(to_rad(MOTOR3_DEGREE))*remap_vx)+(cos(to_rad(MOTOR3_DEGREE))*remap_vy)+(1*remap_wr);

          Serial.print("Motor1_speed: ");Serial.print(motor1_speed);Serial.print("\n");
          Serial.print("Motor2_speed: ");Serial.print(motor2_speed);Serial.print("\n");
          Serial.print("Motor3_speed: ");Serial.print(motor3_speed);Serial.print("\n");
      
          //Set motor 1
          //module_setSpeed(outgoing_frame, 0x01, motor1_speed);
          outgoing_frame.id = 0x014;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor1_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor1_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
          //Set motor 2
          //module_setSpeed(outgoing_frame, 0x02, motor2_speed);
          outgoing_frame.id =  0x024;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor2_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor2_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
          //Set motor 3
          //module_setSpeed(outgoing_frame, 0x03, motor3_speed);
          outgoing_frame.id =  0x034;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor3_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor3_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
        break;

        case 'a'://Left
        Serial.print("Left\n");
          remap_vx = -max_speed;
          remap_vy = 0;
          remap_wr = 0;
          
          //Compute calculation
          motor1_speed = (-sin(to_rad(MOTOR1_DEGREE))*remap_vx)+(cos(to_rad(MOTOR1_DEGREE))*remap_vy)+(1*remap_wr);
          motor2_speed = (-sin(to_rad(MOTOR2_DEGREE))*remap_vx)+(cos(to_rad(MOTOR2_DEGREE))*remap_vy)+(1*remap_wr);
          motor3_speed = (-sin(to_rad(MOTOR3_DEGREE))*remap_vx)+(cos(to_rad(MOTOR3_DEGREE))*remap_vy)+(1*remap_wr);

          Serial.print("Motor1_speed: ");Serial.print(motor1_speed);Serial.print("\n");
          Serial.print("Motor2_speed: ");Serial.print(motor2_speed);Serial.print("\n");
          Serial.print("Motor3_speed: ");Serial.print(motor3_speed);Serial.print("\n");
      
          //Set motor 1
          //module_setSpeed(outgoing_frame, 0x01, motor1_speed);
          outgoing_frame.id = 0x014;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor1_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor1_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
          //Set motor 2
          //module_setSpeed(outgoing_frame, 0x02, motor2_speed);
          outgoing_frame.id =  0x024;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor2_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor2_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
          //Set motor 3
          //module_setSpeed(outgoing_frame, 0x03, motor3_speed);
          outgoing_frame.id =  0x034;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor3_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor3_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
        break;

        case 'd'://Right
        Serial.print("Right\n");
          remap_vx = max_speed;
          remap_vy = 0;
          remap_wr = 0;
          
          //Compute calculation
          motor1_speed = (-sin(to_rad(MOTOR1_DEGREE))*remap_vx)+(cos(to_rad(MOTOR1_DEGREE))*remap_vy)+(1*remap_wr);
          motor2_speed = (-sin(to_rad(MOTOR2_DEGREE))*remap_vx)+(cos(to_rad(MOTOR2_DEGREE))*remap_vy)+(1*remap_wr);
          motor3_speed = (-sin(to_rad(MOTOR3_DEGREE))*remap_vx)+(cos(to_rad(MOTOR3_DEGREE))*remap_vy)+(1*remap_wr);

          Serial.print("Motor1_speed: ");Serial.print(motor1_speed);Serial.print("\n");
          Serial.print("Motor2_speed: ");Serial.print(motor2_speed);Serial.print("\n");
          Serial.print("Motor3_speed: ");Serial.print(motor3_speed);Serial.print("\n");
      
          //Set motor 1
          //module_setSpeed(outgoing_frame, 0x01, motor1_speed);
          outgoing_frame.id = 0x014;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor1_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor1_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
          //Set motor 2
          //module_setSpeed(outgoing_frame, 0x02, motor2_speed);
          outgoing_frame.id =  0x024;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor2_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor2_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
          //Set motor 3
          //module_setSpeed(outgoing_frame, 0x03, motor3_speed);
          outgoing_frame.id =  0x034;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor3_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor3_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
        break;

        case 'q'://Rotate Anti Clockwise
        Serial.print("Rotate Anti Clockwise\n");
          remap_vx = 0;
          remap_vy = 0;
          remap_wr = -max_speed;
          
          //Compute calculation
          motor1_speed = (-sin(to_rad(MOTOR1_DEGREE))*remap_vx)+(cos(to_rad(MOTOR1_DEGREE))*remap_vy)+(1*remap_wr);
          motor2_speed = (-sin(to_rad(MOTOR2_DEGREE))*remap_vx)+(cos(to_rad(MOTOR2_DEGREE))*remap_vy)+(1*remap_wr);
          motor3_speed = (-sin(to_rad(MOTOR3_DEGREE))*remap_vx)+(cos(to_rad(MOTOR3_DEGREE))*remap_vy)+(1*remap_wr);

          Serial.print("Motor1_speed: ");Serial.print(motor1_speed);Serial.print("\n");
          Serial.print("Motor2_speed: ");Serial.print(motor2_speed);Serial.print("\n");
          Serial.print("Motor3_speed: ");Serial.print(motor3_speed);Serial.print("\n");
      
          //Set motor 1
          //module_setSpeed(outgoing_frame, 0x01, motor1_speed);
          outgoing_frame.id = 0x014;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor1_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor1_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
          //Set motor 2
          //module_setSpeed(outgoing_frame, 0x02, motor2_speed);
          outgoing_frame.id =  0x024;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor2_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor2_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
          //Set motor 3
          //module_setSpeed(outgoing_frame, 0x03, motor3_speed);
          outgoing_frame.id =  0x034;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor3_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor3_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
        break;

        case 'e'://rotate clockwiare
        Serial.print("rotate clockwiare\n");
          remap_vx = 0;
          remap_vy = 0;
          remap_wr = max_speed;
          
          //Compute calculation
          motor1_speed = (-sin(to_rad(MOTOR1_DEGREE))*remap_vx)+(cos(to_rad(MOTOR1_DEGREE))*remap_vy)+(1*remap_wr);
          motor2_speed = (-sin(to_rad(MOTOR2_DEGREE))*remap_vx)+(cos(to_rad(MOTOR2_DEGREE))*remap_vy)+(1*remap_wr);
          motor3_speed = (-sin(to_rad(MOTOR3_DEGREE))*remap_vx)+(cos(to_rad(MOTOR3_DEGREE))*remap_vy)+(1*remap_wr);

          Serial.print("Motor1_speed: ");Serial.print(motor1_speed);Serial.print("\n");
          Serial.print("Motor2_speed: ");Serial.print(motor2_speed);Serial.print("\n");
          Serial.print("Motor3_speed: ");Serial.print(motor3_speed);Serial.print("\n");
      
          //Set motor 1
          //module_setSpeed(outgoing_frame, 0x01, motor1_speed);
          outgoing_frame.id = 0x014;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor1_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor1_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
          //Set motor 2
          //module_setSpeed(outgoing_frame, 0x02, motor2_speed);
          outgoing_frame.id =  0x024;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor2_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor2_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
          //Set motor 3
          //module_setSpeed(outgoing_frame, 0x03, motor3_speed);
          outgoing_frame.id =  0x034;
          outgoing_frame.length = 8;
          
          outgoing_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
          outgoing_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
          outgoing_frame.data.byte[2] = (unsigned char) ((motor3_speed>>8) & 0xff);
          outgoing_frame.data.byte[3] = (unsigned char) (motor3_speed  & 0xff);
          outgoing_frame.data.byte[4] = 0x55;
          outgoing_frame.data.byte[5] = 0x55;
          outgoing_frame.data.byte[6] = 0x55;
          outgoing_frame.data.byte[7] = 0x55;
          Can0.sendFrame(outgoing_frame);
          delay(10);
        break;

        
      }
    }
#else
  if(mega_serial.available()){
    //Receive Controller Command
    mega_serial.readBytes(reinterpret_cast<char *>(&mega_serialFrame), sizeof(Speed_cmd));

#ifdef DEBUG_MODE
    Serial.print("Receive PS4");
#endif

    //Start
    int motor1_speed = 0;
    int motor2_speed = 0;
    int motor3_speed = 0;

    int remap_vx = 0;
    int remap_vy = 0;
    int remap_wr = 0;

    switch(mega_serialFrame.cmd){
    case 0x0:
      //Stop
    break;  
    
    
    //Command Move
    case 0x1:
      remap_vx = map(mega_serialFrame.vx, 0, 255, -max_speed, max_speed);
      remap_vy = map(mega_serialFrame.vy, 255, 0, -max_speed, max_speed);
      remap_wr = map(mega_serialFrame.wr, 0, 255, -max_speed, max_speed);
      
      //Compute calculation
      motor1_speed = (-sin(to_rad(MOTOR1_DEGREE))*remap_vx)+(cos(to_rad(MOTOR1_DEGREE))*remap_vy)+(1*remap_wr);
      motor2_speed = (-sin(to_rad(MOTOR2_DEGREE))*remap_vx)+(cos(to_rad(MOTOR2_DEGREE))*remap_vy)+(1*remap_wr);
      motor3_speed = (-sin(to_rad(MOTOR3_DEGREE))*remap_vx)+(cos(to_rad(MOTOR3_DEGREE))*remap_vy)+(1*remap_wr);
  
      
  
       
      //Set motor 1
      module_setSpeed(outgoing_frame, 0x01, motor1_speed);
      Can0.sendFrame(outgoing_frame);
      delay(10);
      //Set motor 2
      module_setSpeed(outgoing_frame, 0x02, motor2_speed);
      Can0.sendFrame(outgoing_frame);
      delay(10);
      //Set motor 3
      module_setSpeed(outgoing_frame, 0x03, motor3_speed);
      Can0.sendFrame(outgoing_frame);
      delay(10);
    break;

    case 0x2:
      //Decrease speed 
      if(max_speed-100 >0)
        max_speed -=100;
      else
        max_speed = 0;
    break;  

    case 0x3:
      //Increase speed
      if(max_speed+100 > 5000)
        max_speed +=100;
      else
        max_speed = 5000;
    break;  
    }//END-Switch Case
  }//END-Mega Serial
#endif

}

inline int to_rad(int in_degree){
  return (in_degree*(PI/180));
}

