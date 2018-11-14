#ifndef ZEDCAM_SERVO_H_
#define ZEDCAM_SERVO_H_
#include "omni_robot_config.h"
#include <Servo.h> 

Servo ServoH;
Servo ServoV;

void servo_setup() {
  ServoH.attach(servoHPin); 
  ServoV.attach(servoVPin);
}

void servoHcal(int percent){
  if (percent < 50){
    ServoH.write((Hmid-Hmin)*percent/50+Hmin);
  }
  else {
    ServoH.write((Hmax-Hmid)*(percent-50)/50+Hmid);
  }
}

void servoVcal(int percent)
{
  if (percent < 50){
    ServoV.write((Vmid-Vmin)*percent/50+Vmin);
  }
  else {
    ServoV.write((Vmax-Vmid)*(percent-50)/50+Vmid);
  }
}


#endif
