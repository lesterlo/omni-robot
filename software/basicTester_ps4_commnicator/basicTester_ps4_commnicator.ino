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

#define DEBUG_MODE
 
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

const int ps4_LeftHatX_deadHigh = 135;
const int ps4_LeftHatX_deadLow = 120;
const int ps4_LeftHatY_deadHigh = 135;
const int ps4_LeftHatY_deadLow = 120;

const int ps4_RightHatX_deadHigh = 135;
const int ps4_RightHatX_deadLow = 120;
const int ps4_RightHatY_deadHigh = 135;
const int ps4_RightHatY_deadLow = 120;

//Variable
float hat_speed_multiply = 0.1;
Speed_cmd outgoing_msg;

char send_char = 0;
char prev_char = 0;


void setup() {
  //Set pin mode

#ifdef DEBUG_MODE
  Serial.begin(115200);
#endif
  due_Serial.begin(115200);



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
    prev_char = send_char;
    send_char = 'c';
  
  //Hat Control
    int leftHatX_value = PS4.getAnalogHat(LeftHatX);
    int leftHatY_value = PS4.getAnalogHat(LeftHatY);
    int rightHatX_value = PS4.getAnalogHat(RightHatX);
    int rightHatY_value = PS4.getAnalogHat(RightHatY);

    //Left Hat X-axis
    if(leftHatX_value < ps4_LeftHatX_deadLow){
      send_char = 'a';
      
    }else if(leftHatX_value > ps4_LeftHatX_deadHigh){
      send_char = 'd';
    }
    //Left Hat Y-axis
    if(leftHatY_value < ps4_LeftHatY_deadLow){
      send_char = 'w';
    }else if(leftHatY_value > ps4_LeftHatX_deadHigh){
      send_char = 's';
    }
    //Right Hat Y-axis
    if(rightHatX_value < ps4_RightHatX_deadLow){
      send_char = 'q';
    }else if(rightHatX_value > ps4_RightHatX_deadHigh){
      send_char = 'e';
    }
    


    //Finally send command
    if(prev_char != send_char){
      due_Serial.print(send_char);
      Serial.print(send_char);
    }
    
    //Disconnect
    if(PS4.getButtonClick(PS)) {
      due_Serial.print('c');
      PS4.disconnect();
    }

  }//END-PS4.Connected
}//END-loop()
