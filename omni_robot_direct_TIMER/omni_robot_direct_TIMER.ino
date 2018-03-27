/*
 * CUHK CSE Omni Robot Control System
 * v1.4
 * Author: Lo Sheung Lai (Lester)
 * 
 * This program is a main control program of the CSE 3 omni wheel robot. It's receive the control signal form the PS4 Controller
 * Please upload it to arduino due with the Omni Robot Control Board
 * 
 * Pleas install the following dependencies
 * 1. Arduino Due board (Tested on 1.6.11)
 * 2. due_can (Tested on 2.0.1)
 * 3. DueTimer (Tested on 1.4.7)
 * 4. USB Host Shield 2.0 (Tested on 1.3.2)
 */
#include "omni_robot_config.h"
//Global Variable, don't touch
volatile int pwm_speed = MAX_PWM;
int cur_speed = DEFAULT_RPM;
//bool stop_flag = false;
//Internal Use
int remap_vx = 0;
int remap_vy = 0;
int remap_wr = 0;
int speed_adder = 0;
int speed_adder_l2 = 0;
int speed_adder_r2 = 0;
//ps4 value
int leftHatX = 0;
int leftHatY = 0;
int rightHatX = 0;
int l2ButVal = 0;
int r2ButVal = 0;
volatile int motor1_speed = 0;
volatile int motor2_speed = 0;
volatile int motor3_speed = 0;

#ifdef CAL_LUT_MODE
const int PRECISION_MULTIPYER = 1000;
static int lut_motor_sin[3];
static int lut_motor_cos[3];
void compute_lut(){
    //sin port
    lut_motor_sin[0] = (-sin(TO_RAD(MOTOR1_DEGREE)))*PRECISION_MULTIPYER;
    lut_motor_sin[1] = (-sin(TO_RAD(MOTOR2_DEGREE)))*PRECISION_MULTIPYER;
    lut_motor_sin[2] = (-sin(TO_RAD(MOTOR3_DEGREE)))*PRECISION_MULTIPYER;
    //cos part
    lut_motor_cos[0] = cos(TO_RAD(MOTOR1_DEGREE))*PRECISION_MULTIPYER;
    lut_motor_cos[1] = cos(TO_RAD(MOTOR2_DEGREE))*PRECISION_MULTIPYER;
    lut_motor_cos[2] = cos(TO_RAD(MOTOR3_DEGREE))*PRECISION_MULTIPYER; 
}
#endif

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

#ifdef CAL_LUT_MODE
  compute_lut();
#endif
}
void loop() 
{
  //1. preparation
  //2. PS4 Logic
  Usb.Task();//Call for doing the USB jobs
  if (PS4.connected())
  {
    //2.1 - Get PS4 Controller input
    r2ButVal = PS4.getAnalogButton(R2); //Boost button
    l2ButVal = PS4.getAnalogButton(L2);
    if(r2ButVal){// Value >0
      speed_adder_r2 = r2ButVal*SPEED_ACCEL_INV;//Add speed
    }else {
      speed_adder_r2 = 0;
    }
    if(l2ButVal){// Value >0
      speed_adder_l2 = -(l2ButVal*SPEED_ACCEL_INV);//Subtract speed
    }else {
      speed_adder_l2 = 0;
    }
    speed_adder = speed_adder_l2+speed_adder_r2; //Max 2 button value
    
    //Get Joystick Position Value
    leftHatX = PS4.getAnalogHat(LeftHatX);
    leftHatY = PS4.getAnalogHat(LeftHatY);
    rightHatX = PS4.getAnalogHat(RightHatX);
    //int rightHatY = PS4.getAnalogHat(RightHatY);
    //2.2 Check valid control action
    //2.3a - Robot X-axis speed  
    if(leftHatX > PS4_LEFTX_UPPER_DZ || leftHatX  < PS4_LEFTX_LOWER_DZ) //Check the pointer is out of the DeadZone
    {
      //NormalSpeed Map
      remap_vx = map(leftHatX, PS4_HAT_MAX_VAL, PS4_HAT_MIN_VAL, -(cur_speed+speed_adder), (cur_speed+speed_adder)); //Flip the Axis direction

      //Speed Divided Map
      //remap_vx = map(leftHatX, PS4_HAT_MIN_VAL, PS4_HAT_MAX_VAL, -cur_speed, cur_speed); 
      //remap_vx /= VX_SPEED_DIVIDER;//Reduce the value
    }else
    {
      remap_vx = 0;//Stop Move
    }
    //2.3b - Robot Y-axis speed 
    if(leftHatY > PS4_LEFTY_UPPER_DZ || leftHatY < PS4_LEFTX_LOWER_DZ) //Check the pointer is out of the DeadZone
    {
      remap_vy = map(leftHatY, PS4_HAT_MIN_VAL, PS4_HAT_MAX_VAL, -(cur_speed+speed_adder), (cur_speed+speed_adder));//No need to Flip
      //remap_vy /= VY_SPEED_DIVIDER;//Reduce the value
    }else
    {
      remap_vy = 0;//Stop Move
    }
    //2.3c - Robot Rotation speed  
    if(rightHatX > PS4_RIGHTX_UPPER_DZ || rightHatX < PS4_RIGHTX_LOWER_DZ) //Check the pointer is out of the DeadZone
    {
      remap_wr = map(rightHatX, PS4_HAT_MAX_VAL, PS4_HAT_MIN_VAL, -(cur_speed+speed_adder), (cur_speed+speed_adder));//Flip the Axis direction
      //remap_wr = map(rightHatX, PS4_HAT_MIN_VAL, PS4_HAT_MAX_VAL, -cur_speed, cur_speed);
      //remap_wr /= WR_SPEED_DIVIDER;//Reduce the value
    }else
    {
      remap_wr = 0; 
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
  if(remap_vx == 0 && remap_vy == 0 && remap_wr == 0)
  {
    //ignore the motor speed matrix calculation when it should stop (speed=0)
    noInterrupts();//Critical State, must complete all of the calculate before the interrupt called
    motor1_speed = 0;
    motor2_speed = 0;
    motor3_speed = 0;
    interrupts();
  }
  else
  {
    //Matrix calculation
    noInterrupts(); //Critical State, must complete all of the calculate before the interrupt called
#ifdef CAL_LUT_MODE
    motor1_speed = (lut_motor_sin[0] * remap_vx / PRECISION_MULTIPYER)+(lut_motor_cos[0]*remap_vy / PRECISION_MULTIPYER)+ remap_wr;
    motor2_speed = (lut_motor_sin[1] * remap_vx / PRECISION_MULTIPYER)+(lut_motor_cos[1]*remap_vy / PRECISION_MULTIPYER)+ remap_wr;
    motor3_speed = (lut_motor_sin[2] * remap_vx / PRECISION_MULTIPYER)+(lut_motor_cos[2]*remap_vy / PRECISION_MULTIPYER)+ remap_wr; 
#else    
    motor1_speed = (-sin(TO_RAD(MOTOR1_DEGREE))*remap_vx)+(cos(TO_RAD(MOTOR1_DEGREE))*remap_vy)+(1*remap_wr);
    motor2_speed = (-sin(TO_RAD(MOTOR2_DEGREE))*remap_vx)+(cos(TO_RAD(MOTOR2_DEGREE))*remap_vy)+(1*remap_wr);
    motor3_speed = (-sin(TO_RAD(MOTOR3_DEGREE))*remap_vx)+(cos(TO_RAD(MOTOR3_DEGREE))*remap_vy)+(1*remap_wr);
#endif
    interrupts();
  }//END-3
  
#ifdef DEBUG_MODE
  Serial3.print("vx: ");
  Serial3.println(remap_vx);
  Serial3.print("vy ");
  Serial3.println(remap_vy);
  Serial3.print("wr: ");
  Serial3.println(remap_wr);
  Serial3.print("M1: ");
  Serial3.println(motor1_speed);
  Serial3.print("M2: ");
  Serial3.println(motor2_speed);
  Serial3.print("M3: ");
  Serial3.println(motor3_speed);
  Serial3.println(" ");
#endif
  //Send the motor signal, in polling
//  motor1_setSpeed(pwm_speed, motor1_speed);
//  motor2_setSpeed(pwm_speed, motor2_speed);
//  motor3_setSpeed(pwm_speed, motor3_speed);
//  delay(1);// delay aleas 1 ms
}//END-loop

