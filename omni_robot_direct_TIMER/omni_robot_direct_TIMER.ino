/*
 * CUHK CSE Omni Robot Control System
 * 
 * This program is a main control program of the CSE 3 omni wheel robot. It's receive the control signal form the PS4 Controller
 * Please upload it to arduino due with the Omni Robot Control Board
 */
 
//#define DEBUG_MODE


#define TO_RAD(IN_DEGREE)(IN_DEGREE*(PI/180))
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
const int MIN_RPM = 50;
const int DEFAULT_RPM = 500;

const int SPEED_CHANGE_INV = 150;

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

//Global Variable, don't touch
volatile int pwm_speed = MAX_PWM;
int cur_speed = DEFAULT_RPM;
//bool stop_flag = false;
//Internal Use
int remap_vx = 0;
int remap_vy = 0;
int remap_wr = 0;
int leftHatX = 0;
int leftHatY = 0;
int rightHatX = 0;
volatile int motor1_speed = 0;
volatile int motor2_speed = 0;
volatile int motor3_speed = 0;

void send_MotorData(){ //Call every 1ms
  motor1_setSpeed(pwm_speed, motor1_speed);
  motor2_setSpeed(pwm_speed, motor2_speed);
  motor3_setSpeed(pwm_speed, motor3_speed);  
}

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
    Serial3.begin(115200);
//CAN module init
  pinMode(CANBUS_ENABLE_PIN, OUTPUT);
  digitalWrite(CANBUS_ENABLE_PIN, LOW); //Enable the CANBUS chip
  delay(500);
  
  Can0.begin(CAN_BPS_500K); //Setup CAN bus, the author said 100K is not working
  Can0.setNumTXBoxes(6); //Need aleast 3 Transmit buffer, because there have 3 motor driver
  
  //CAN bus setup
  reset_module(0x0); //reset ALL module
  delay(500); 
  module_setSpeedMode(0x0); //Set speed mode to ALL module
  delay(500);
  
  
#ifdef DEBUG_MODE
  Serial.print("Omni Wheel Robot Start\n");
#endif

//Debug LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

//Setup Timer
  Timer3.attachInterrupt(send_MotorData);
  Timer3.start(1000); // Calls every 1ms
}
void loop() 
{
  //1. preparation
  //2. PS4 Logic
  Usb.Task();//Call for doing the USB jobs
  if (PS4.connected())
  {
    //2.1 - Get PS4 Controller input
    //Get Joystick Position Value
    leftHatX = PS4.getAnalogHat(LeftHatX);
    leftHatY = PS4.getAnalogHat(LeftHatY);
    rightHatX = PS4.getAnalogHat(RightHatX);
    //int rightHatY = PS4.getAnalogHat(RightHatY);
    //2.2 Check valid control action
    //2.3a - Robot X-axis speed
    if(leftHatX > PS4_LEFTX_UPPER_DZ || leftHatX  < PS4_LEFTX_LOWER_DZ) //Check the pointer is out of the DeadZone
    {
      remap_vx = map(leftHatX, 255, 0, -cur_speed, cur_speed); //Flip the Axis direction
      remap_vx /= VX_SPEED_DIVIDER;//Reduce the value
    }else
    {
      remap_vx = 127;//Stop Move
    }
    //2.3b - Robot Y-axis speed 
    if(leftHatY > PS4_LEFTY_UPPER_DZ || leftHatY < PS4_LEFTX_LOWER_DZ) //Check the pointer is out of the DeadZone
    {
      remap_vy = map(leftHatY, 0, 255, -cur_speed, cur_speed);//No need to Flip
      remap_vy /= VY_SPEED_DIVIDER;//Reduce the value
    }else
    {
      remap_vy = 127;//Stop Move
    }
    //2.3c - Robot Rotation speed  
    if(rightHatX > PS4_RIGHTX_UPPER_DZ || rightHatX < PS4_RIGHTX_LOWER_DZ) //Check the pointer is out of the DeadZone
    {
      remap_wr = map(rightHatX, 255, 0, -cur_speed, cur_speed);//Flip the Axis direction
      remap_wr /= WR_SPEED_DIVIDER;//Reduce the value
    }else
    {
      remap_wr = 127; 
    }
    //2.3d - useless
    //if(rightHatY > PS4_RIGHTY_UPPER_DZ || rightHatY < PS4_RIGHTY_LOWER_DZ )

    //3. Button Action
    if(PS4.getButtonClick(R1)){ //Increase speed
      //Speed setting protection, not pass the maximum value bound
      if((cur_speed+SPEED_CHANGE_INV) >= MAX_RPM)
        cur_speed = MAX_RPM;
      else
        cur_speed += SPEED_CHANGE_INV;
    }
    if(PS4.getButtonClick(L1)){ //Decrease speed
      //Speed setting protection, not pass the minimum value bound
      if((cur_speed-SPEED_CHANGE_INV) <= MIN_RPM)
        cur_speed = MIN_RPM;
      else
        cur_speed -= SPEED_CHANGE_INV;
    }
    if(PS4.getButtonClick(TOUCHPAD))
      cur_speed = DEFAULT_RPM;    
    //5. PS4 Exit State
    if (PS4.getButtonClick(PS)) 
      PS4.disconnect();

    if (PS4.getButtonClick(CROSS)) {
      PS4.setLedFlash(10, 10); // Set it to blink rapidly
    }
    if (PS4.getButtonClick(SQUARE)) {
        PS4.setLedFlash(0, 0); // Turn off blinking
      }
  }//END-2
  //3 - Compute Logic

  //ignore the motor speed matrix calculation when it should stop
  if(remap_vx == 127 && remap_vy == 127 && remap_wr == 127)
  {
    noInterrupts();//Critical State, must complete all of the calculate before the interrupt called
    motor1_speed = 0;
    motor2_speed = 0;
    motor3_speed = 0;
    interrupts();
  }else
  {
    //4 - Matrix calculation
    noInterrupts(); //Critical State, must complete all of the calculate before the interrupt called
    motor1_speed = (-sin(TO_RAD(MOTOR1_DEGREE))*remap_vx)+(cos(TO_RAD(MOTOR1_DEGREE))*remap_vy)+(1*remap_wr);
    motor2_speed = (-sin(TO_RAD(MOTOR2_DEGREE))*remap_vx)+(cos(TO_RAD(MOTOR2_DEGREE))*remap_vy)+(1*remap_wr);
    motor3_speed = (-sin(TO_RAD(MOTOR3_DEGREE))*remap_vx)+(cos(TO_RAD(MOTOR3_DEGREE))*remap_vy)+(1*remap_wr);
    interrupts();
  }
  
#ifdef DEBUG_MODE
  Serial3.print("Motor 1: ");
  Serial3.println(motor1_speed);
  Serial3.print("Motor 2: ");
  Serial3.println(motor2_speed);
  Serial3.print("Motor 3: ");
  Serial3.println(motor3_speed);
#endif
  //Send the motor signal, in polling
//  motor1_setSpeed(pwm_speed, motor1_speed);
//  motor2_setSpeed(pwm_speed, motor2_speed);
//  motor3_setSpeed(pwm_speed, motor3_speed);
//  delay(1);// delay aleas 1 ms
}//END-loop

