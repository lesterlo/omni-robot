/*
 * CUHK CSE Omni Robot Control System
 * 
 * This program is a main control program of the CSE 3 omni wheel robot. It's receive the control signal form the PS4 Controller
 * Please upload it to arduino due with the Omni Robot Control Board
 */
#define DEBUG_MODE
#define TO_RAD(IN_DEGREE)(IN_DEGREE*(PI/180))
#define WHEEL_DIAMETER 0.25
#define MS2RPM(IN_SPEED) (int)((IN_SPEED*60/(2*PI*WHEEL_DIAMETER))*10) //Multiplier

//Const Value
const int ROBOT_FRONT_VECTOR = 90;

//Controller Setting
const int PS4_LEFTX_UPPER_DZ = 137; 
const int PS4_LEFTX_LOWER_DZ = 117;
const int PS4_LEFTY_UPPER_DZ = 137; 
const int PS4_LEFTY_LOWER_DZ = 117;
const int PS4_RIGHTX_UPPER_DZ = 137; 
const int PS4_RIGHTX_LOWER_DZ = 117;
const int PS4_RIGHTY_UPPER_DZ = 137; 
const int PS4_RIGHTY_LOWER_DZ = 117;

//Motor Seperation
const int MOTOR1_DEGREE = 0+ROBOT_FRONT_VECTOR;
const int MOTOR2_DEGREE = 120+ROBOT_FRONT_VECTOR;
const int MOTOR3_DEGREE = 240+ROBOT_FRONT_VECTOR;

//MOTOR Driver Setting
const int WR_SPEED_DIVIDER = 2; //Reduce the rotation speed
const int VX_SPEED_DIVIDER = 1; //Reduct the X-axis speed
const int VY_SPEED_DIVIDER = 1; //Reduct the Y-axis speed

const int MAX_PWM = 5000;
const int MAX_RPM = 2000;
const int DEFAULT_RPM = 500;

const int SPEED_CHANGE_INV = 100;

//Pin Define
const int CANBUS_ENABLE_PIN = 23;

//Include Library
#include "robomodule_direct_lib.h"
#include "due_can.h"
//USB host sheild 2.0 library include
#include <PS4BT.h>
#include <usbhub.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
PS4BT PS4(&Btd);

//Global Variable, don't touch
bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;

int pwm_speed = MAX_PWM;
int cur_speed = DEFAULT_RPM;


void setup() {
//USB host shield code
#ifdef DEBUG_MODE
  Serial.begin(115200);
#endif
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
    while(Usb.Init() == -1){// == -1
      Serial.print(F("\r\nOSC did not start"));
      delay(1000);// Reconnect after wait for 1s.
    }
    
//CAN module init
  pinMode(CANBUS_ENABLE_PIN, OUTPUT);
  digitalWrite(CANBUS_ENABLE_PIN, LOW); //Enable the CANBUS chip
  delay(500);
  Can0.begin(CAN_BPS_1000K); //Setup CAN bus
  //CAN bus setup
  reset_module(0x0); //reset ALL module
  delay(500); 
  module_setSpeedMode(0x0); //Set speed mode to ALL module
  delay(500);
#ifdef DEBUG_MODE
  Serial.print("Omni Wheel Robot Start\n");
#endif
}
void loop() 
{
  //1. preparation
  //2. PS4 Logic
  Usb.Task();
  if (PS4.connected())
  {
    int remap_vx = 0;
    int remap_vy = 0;
    int remap_wr = 0;
    int motor1_speed = 0;
    int motor2_speed = 0;
    int motor3_speed = 0;
    //2.1 - Get PS4 Controller input
    //Get Value
    int leftHatX = PS4.getAnalogHat(LeftHatX);
    int leftHatY = PS4.getAnalogHat(LeftHatY);
    int rightHatX = PS4.getAnalogHat(RightHatX);
    //int rightHatY = PS4.getAnalogHat(RightHatY);
    //2.2 Check valid control action
    //2.3a - Robot X-axis speed
    if(leftHatX > PS4_LEFTX_UPPER_DZ || leftHatX  < PS4_LEFTX_LOWER_DZ)
    {
      remap_vx = map(leftHatX, 0, 255, -cur_speed, cur_speed);
    }else
    {
      remap_vx = 0;//Stop Move
    }
    //2.3b - Robot Y-axis speed 
    if(leftHatY > PS4_LEFTY_UPPER_DZ || leftHatY < PS4_LEFTX_LOWER_DZ)
    {
      remap_vy = map(leftHatY, 0, 255, -cur_speed, cur_speed); 
    }else
    {
      remap_vy = 0;//Stop Move
    }
    //2.3c - Robot Rotation speed  
    if(rightHatX > PS4_RIGHTX_UPPER_DZ || rightHatX < PS4_RIGHTX_LOWER_DZ)
    {
      remap_wr = map(rightHatX, 0, 255, -cur_speed, cur_speed);
    }else
    {
      remap_wr = 0; 
    }
    //2.3d - useless
    //if(rightHatY > PS4_RIGHTY_UPPER_DZ || rightHatY < PS4_RIGHTY_LOWER_DZ )

    remap_vx /= VX_SPEED_DIVIDER;
    remap_vy /= VY_SPEED_DIVIDER;
    remap_wr /= WR_SPEED_DIVIDER;
    //3 - Matrix calculation
    motor1_speed = (-sin(TO_RAD(MOTOR1_DEGREE))*remap_vx)+(cos(TO_RAD(MOTOR1_DEGREE))*remap_vy)+(1*remap_wr);
    motor2_speed = (-sin(TO_RAD(MOTOR2_DEGREE))*remap_vx)+(cos(TO_RAD(MOTOR2_DEGREE))*remap_vy)+(1*remap_wr);
    motor3_speed = (-sin(TO_RAD(MOTOR3_DEGREE))*remap_vx)+(cos(TO_RAD(MOTOR3_DEGREE))*remap_vy)+(1*remap_wr);
  
    motor1_setSpeed(pwm_speed, motor1_speed);
    motor2_setSpeed(pwm_speed, motor2_speed);
    motor3_setSpeed(pwm_speed, motor3_speed);
    
  }//END-2
  //3. Speed Control
  if(PS4.getButtonPress(R1)) //Increase speed
    cur_speed = (cur_speed+SPEED_CHANGE_INV > MAX_RPM)? cur_speed+SPEED_CHANGE_INV : MAX_RPM;
  if(PS4.getButtonPress(L1)) //Decrease speed
    cur_speed = (cur_speed+SPEED_CHANGE_INV > 0)? cur_speed-SPEED_CHANGE_INV : 0;
  if(PS4.getButtonPress(TOUCHPAD))
    cur_speed = DEFAULT_RPM;    
  //5. PS4 Exit State
  if (PS4.getButtonClick(PS)) 
    PS4.disconnect();
}//END-loop

