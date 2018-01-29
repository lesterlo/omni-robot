#include "robomodule_direct_lib.h"
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

int max_speed = 500;
int pwm_speed = 5000;

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
  digitalWrite(CANBUS_ENABLE_PIN, LOW); //Enable the CANBUS chip


  
  //CAN bus setup
  reset_module(0x0);
  delay(500);
  
  module_setSpeedMode(0x0); //Set Mode
  delay(500);

}

void loop() {

  if(mega_serial.available()){
    //Receive Controller Command
    mega_serial.readBytes(reinterpret_cast<char *>(&mega_serialFrame), sizeof(Speed_cmd));

#ifdef DEBUG_MODE
    Serial.print("Receive PS4");
    Serial.print("vx=");Serial.print(mega_serialFrame.vx);Serial.print("\n");
    Serial.print("vy=");Serial.print(mega_serialFrame.vy);Serial.print("\n");
    Serial.print("wr=");Serial.print(mega_serialFrame.wr );Serial.print("\n");
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
      if(mega_serialFrame.vx==127 || mega_serialFrame.vy==127 || mega_serialFrame.wr==127)
        break;
      remap_vx = map(mega_serialFrame.vx, 0, 255, -max_speed, max_speed);
      remap_vy = map(mega_serialFrame.vy, 255, 0, -max_speed, max_speed);
      remap_wr = map(mega_serialFrame.wr, 0, 255, -max_speed, max_speed);

#ifdef DEBUG_MODE
    Serial.print("Receive remap");
    Serial.print("rvx=");Serial.print(remap_vx);Serial.print("\n");
    Serial.print("rvy=");Serial.print(remap_vy);Serial.print("\n");
    Serial.print("rwr=");Serial.print(remap_wr);Serial.print("\n");
#endif
      if(mega_serialFrame.vx==127 || mega_serialFrame.vy==127 || mega_serialFrame.wr==127){
        remap_vx=0;
        remap_vy=0;
        remap_wr=0;
      }
      //Compute calculation
      motor1_speed = (-sin(to_rad(MOTOR1_DEGREE))*remap_vx)+(cos(to_rad(MOTOR1_DEGREE))*remap_vy)+(1*remap_wr);
      motor2_speed = (-sin(to_rad(MOTOR2_DEGREE))*remap_vx)+(cos(to_rad(MOTOR2_DEGREE))*remap_vy)+(1*remap_wr);
      motor3_speed = (-sin(to_rad(MOTOR3_DEGREE))*remap_vx)+(cos(to_rad(MOTOR3_DEGREE))*remap_vy)+(1*remap_wr);
  
      
  
       
      //Set motor 1
      motor1_setSpeed(pwm_speed, motor1_speed);
      motor2_setSpeed(pwm_speed, motor2_speed);
      motor3_setSpeed(pwm_speed, motor3_speed);
     
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

}



inline int to_rad(int in_degree){
  return (in_degree*(PI/180));
}

