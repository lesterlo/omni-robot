/*
 * CUHK CSE Omni Robot Control System
 * v1.4
 * Author: Lo Sheung Lai (Lester)
 * 
 * Section - Config file
 * 
 * 
 * This program is a main control program of the CSE 3 omni wheel robot. It's receive the control signal form the PS4 Controller
 * Please upload it to arduino due with the Omni Robot Control Board
 */

#ifndef OMNI_ROBOT_CONFIG_H
#define OMNI_ROBOT_CONFIG_H

//Build option
//#define DEBUG_MODE
#define CAL_LUT_MODE

//Marco Function
#define TO_RAD(IN_DEGREE) (IN_DEGREE*(PI/180))
#define WHEEL_DIAMETER 0.25
#define MS2RPM(IN_SPEED) (int)((IN_SPEED*60/(2*PI*WHEEL_DIAMETER))*10) //Multiplier

//Const Value
const int ROBOT_FRONT_VECTOR = -30;//Prefect Number!! away from the motor in 90 degree anti clockwise

//Controller Setting
const int PS4_LEFTX_UPPER_DZ = 137; 
const int PS4_LEFTX_LOWER_DZ = 117;
const int PS4_LEFTY_UPPER_DZ = 137; 
const int PS4_LEFTY_LOWER_DZ = 117;
const int PS4_RIGHTX_UPPER_DZ = 137; 
const int PS4_RIGHTX_LOWER_DZ = 117;
const int PS4_RIGHTY_UPPER_DZ = 137; 
const int PS4_RIGHTY_LOWER_DZ = 117;
const int PS4_HAT_MAX_VAL = 254;
const int PS4_HAT_MIN_VAL = 1;

//Motor Seperation
const int MOTOR1_DEGREE = 0+ROBOT_FRONT_VECTOR;
const int MOTOR2_DEGREE = 120+ROBOT_FRONT_VECTOR;
const int MOTOR3_DEGREE = 240+ROBOT_FRONT_VECTOR;

//MOTOR Driver Setting
//const int WR_SPEED_DIVIDER = 1; //Reduce the rotation speed
//const int VX_SPEED_DIVIDER = 1; //Reduct the X-axis speed
//const int VY_SPEED_DIVIDER = 1; //Reduct the Y-axis speed
//const int MOTOR_SPEED_DIVIDER = 1; //Reduct the Y-axis speed

const int MAX_PWM = 5000;
const int MAX_RPM = 2000;
const int MIN_RPM = 50;
const int DEFAULT_RPM = 500;
const int SPEED_CHANGE_INV = 150;
const int SPEED_ACCEL_INV = 2;

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

//Include due timer
#include <DueTimer.h>

#endif
