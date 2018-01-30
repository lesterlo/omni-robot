#ifndef ROBOMODULE_DIRECT_LIB_
#define ROBOMODULE_DIRECT_LIB_

#include "due_can.h"
#include <stdint.h>

void reset_All_module(){
  CAN_FRAME can_frame;
  can_frame.id = 0x000;
  can_frame.length = 8;
  
  can_frame.data.byte[0] = 0x55;
  can_frame.data.byte[1] = 0x55;
  can_frame.data.byte[2] = 0x55;
  can_frame.data.byte[3] = 0x55;
  can_frame.data.byte[4] = 0x55;
  can_frame.data.byte[5] = 0x55;
  can_frame.data.byte[6] = 0x55;
  can_frame.data.byte[7] = 0x55;

  Can0.sendFrame(can_frame);
  delay(10);
}

void reset_module(uint8_t can_ID){
  CAN_FRAME can_frame;
  can_frame.id = (can_ID<<8) +0x0;
  can_frame.length = 8;
  
  can_frame.data.byte[0] = 0x55;
  can_frame.data.byte[1] = 0x55;
  can_frame.data.byte[2] = 0x55;
  can_frame.data.byte[3] = 0x55;
  can_frame.data.byte[4] = 0x55;
  can_frame.data.byte[5] = 0x55;
  can_frame.data.byte[6] = 0x55;
  can_frame.data.byte[7] = 0x55;

  Can0.sendFrame(can_frame);
  delay(10);
}

void module_setSpeedMode(uint16_t can_ID){
  CAN_FRAME can_frame;
  //can_frame.id = (can_ID<<8) +0x1;
  can_frame.id = 0x001;
  can_frame.length = 8;
  
  can_frame.data.byte[0] = 0x03; //Speed Mode
  can_frame.data.byte[1] = 0x55;
  can_frame.data.byte[2] = 0x55;
  can_frame.data.byte[3] = 0x55;
  can_frame.data.byte[4] = 0x55;
  can_frame.data.byte[5] = 0x55;
  can_frame.data.byte[6] = 0x55;
  can_frame.data.byte[7] = 0x55;

  Can0.sendFrame(can_frame);
  delay(10);
}

void motor1_setSpeed(uint16_t pwm_speed, uint16_t motor_speed){
  CAN_FRAME can_frame;
  can_frame.id =  0x014;
  can_frame.length = 8;
  
  can_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
  can_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
  can_frame.data.byte[2] = (unsigned char) ((motor_speed>>8) & 0xff);
  can_frame.data.byte[3] = (unsigned char) (motor_speed  & 0xff);
  can_frame.data.byte[4] = 0x55;
  can_frame.data.byte[5] = 0x55;
  can_frame.data.byte[6] = 0x55;
  can_frame.data.byte[7] = 0x55;
  Can0.sendFrame(can_frame);
}

void motor2_setSpeed(uint16_t pwm_speed, uint16_t motor_speed){
  CAN_FRAME can_frame;
  can_frame.id =  0x024;
  can_frame.length = 8;
  
  can_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
  can_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
  can_frame.data.byte[2] = (unsigned char) ((motor_speed>>8) & 0xff);
  can_frame.data.byte[3] = (unsigned char) (motor_speed  & 0xff);
  can_frame.data.byte[4] = 0x55;
  can_frame.data.byte[5] = 0x55;
  can_frame.data.byte[6] = 0x55;
  can_frame.data.byte[7] = 0x55;
  Can0.sendFrame(can_frame);
}

void motor3_setSpeed(uint16_t pwm_speed, uint16_t motor_speed){
  CAN_FRAME can_frame;
  can_frame.id =  0x034;
  can_frame.length = 8;
  
  can_frame.data.byte[0] = (unsigned char) ((pwm_speed >>8) & 0xff);
  can_frame.data.byte[1] = (unsigned char) ( pwm_speed & 0xff);
  can_frame.data.byte[2] = (unsigned char) ((motor_speed>>8) & 0xff);
  can_frame.data.byte[3] = (unsigned char) (motor_speed  & 0xff);
  can_frame.data.byte[4] = 0x55;
  can_frame.data.byte[5] = 0x55;
  can_frame.data.byte[6] = 0x55;
  can_frame.data.byte[7] = 0x55;
  Can0.sendFrame(can_frame);
}

#endif
