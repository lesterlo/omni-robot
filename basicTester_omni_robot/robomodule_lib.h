#ifndef ROBOMODULE_LIB_
#define ROBOMODULE_LIB_

#include "due_can.h"
#include <stdint.h>

//Const Parameter
typedef enum {
  RESET_MODE=0x0,
  OPENLOOP_MODE=0x1,
  CURRENT_MODE=0x2,
  SPEED_MODE=0x3,
  POSITION_MODE=0x4,
  SPEED_POSITION_MODE=0x5,
  CURRENT_SPEED_MODE=0x6,
  CURRENT_POSITION_MODE=0x7,
  CURRENT_SPEED_POSTION_MODE=0x8
  } MODULE_MODE;

//Variable
MODULE_MODE current_mode = RESET_MODE;
short max_PWM = 0;




//Function
void module_reset(CAN_FRAME& can_frame, uint16_t can_id){
  can_frame.id = (can_id << 8) + RESET_MODE;
  can_frame.length = 8;
  can_frame.data.byte[0] = 0x55;
  can_frame.data.byte[1] = 0x55;
  can_frame.data.byte[2] = 0x55;
  can_frame.data.byte[3] = 0x55;
  can_frame.data.byte[4] = 0x55;
  can_frame.data.byte[5] = 0x55;
  can_frame.data.byte[6] = 0x55;
  can_frame.data.byte[7] = 0x55;
  
}

void module_setMode(CAN_FRAME& can_frame, uint16_t can_id, MODULE_MODE in_mode){
  current_mode = in_mode;
  can_frame.id = (can_id << 8) + 0x1;
  can_frame.length = 8;
  
  can_frame.data.byte[0] = in_mode;
  can_frame.data.byte[1] = 0x55;
  can_frame.data.byte[2] = 0x55;
  can_frame.data.byte[3] = 0x55;
  can_frame.data.byte[4] = 0x55;
  can_frame.data.byte[5] = 0x55;
  can_frame.data.byte[6] = 0x55;
  can_frame.data.byte[7] = 0x55;
}

void set_Max_PWM(short in_value){
  max_PWM = in_value;
}

void module_setSpeed(CAN_FRAME& can_frame, uint16_t can_id, int input_Velocity){
  can_frame.id = (can_id << 8) + 0x4;
  can_frame.length = 8;
  
  can_frame.data.byte[0] = (unsigned char) ((max_PWM >>8) & 0xff);
  can_frame.data.byte[1] = (unsigned char) (max_PWM & 0xff);
  can_frame.data.byte[2] = (unsigned char) ((input_Velocity>>8) & 0xff);
  can_frame.data.byte[3] = (unsigned char) (input_Velocity  & 0xff);
  can_frame.data.byte[4] = 0x55;
  can_frame.data.byte[5] = 0x55;
  can_frame.data.byte[6] = 0x55;
  can_frame.data.byte[7] = 0x55;
}



#endif
