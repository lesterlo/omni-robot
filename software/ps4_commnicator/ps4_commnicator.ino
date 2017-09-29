/*
 * Omni Robot PS4 controller communicator
 * Program to the arduino mega board
 *  Version: 1.0
 * 
 *  Author: Lo Sheung Lai
 *  Last Modified: 28/9/2017, 11:08 pm
 *  
 */
#include "commu_data_exchange.h"
 
HardwareSerial &due_Serial = Serial3;

//PS4 Remote logic
#include <PS4BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);

const int ps4_LeftHatX_deadHigh = 150;
const int ps4_LeftHatX_deadLow = 100;
const int ps4_LeftHatY_deadHigh = 150;
const int ps4_LeftHatY_deadLow = 100;

const int ps4_RightHatX_deadHigh = 137;
const int ps4_RightHatX_deadLow = 117;
const int ps4_RightHatY_deadHigh = 137;
const int ps4_RightHatY_deadLow = 117;

//Variable
float hat_speed_multiply = 0.1;
Speed_cmd outgoing_msg;


void setup() {
  //Set pin mode

#ifdef DEBUG_MODE
  Serial.begin(115200);
#else
  due_Serial.begin(115200);
#endif


  //Connect to PS4 Remote
  if (Usb.Init() == -1) {
#ifdef DEBUG_MODE
    Serial.print(F("\r\nOSC did not start"));
#endif
    while (1); // Halt
  }
  
  PS4.setLed(Red);


}

void loop() {
  Usb.Task();
  if(PS4.connected()){//Check the ps4 remote is connected
    if(PS4.getButtonClick(TOUCHPAD)){

    }
  
  //Hat Control
    int leftHatX_value = PS4.getAnalogHat(LeftHatX);
    int leftHatY_value = PS4.getAnalogHat(LeftHatY);
    int rightHatX_value = PS4.getAnalogHat(RightHatX);
    int rightHatY_value = PS4.getAnalogHat(RightHatY);

    //Left Hat X-axis
    if(leftHatX_value < ps4_LeftHatX_deadLow){
      outgoing_msg.vx = leftHatX_value; //Negative, Maybe need conversion
      
    }else if(leftHatX_value > ps4_LeftHatX_deadHigh){
      outgoing_msg.vx = leftHatX_value; //Positive, Maybe need conversion
    }else{
       outgoing_msg.vx = 0;
    }
    //Left Hat Y-axis
    if(leftHatY_value < ps4_LeftHatY_deadLow){
      outgoing_msg.vy = leftHatY_value; //Negative, Maybe need conversion
    }else if(leftHatY_value > ps4_LeftHatX_deadHigh){
      outgoing_msg.vx = leftHatY_value; //Positive, Maybe need conversion
    }else{
      outgoing_msg.vy = 0;
    }
    //Left Hat Y-axis
    if(rightHatX_value < ps4_RightHatX_deadLow){
      outgoing_msg.wr = rightHatX_value; //Negative, Maybe need conversion
    }else if(rightHatX_value > ps4_RightHatX_deadHigh){
      outgoing_msg.wr = rightHatX_value; //Positive, Maybe need conversion
    }else{
      outgoing_msg.wr = 0;
    }

    if (PS4.getButtonClick(L1)){
      if(hat_speed_multiply > 0.1)
        hat_speed_multiply -=0.1;
    }

    if (PS4.getButtonClick(R1)){
      if(hat_speed_multiply <= 0.7)
        hat_speed_multiply +=0.1;
    }

    due_Serial.write(reinterpret_cast<char *>(&outgoing_msg), sizeof(Speed_cmd));
 
    //Disconnect
    if(PS4.getButtonClick(PS)) {
      PS4.disconnect();
    }

  }//END-PS4.Connected
}//END-loop()
