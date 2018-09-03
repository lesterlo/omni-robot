//Hobbywing Motor Tester
#include <Servo.h>

Servo bldc1;

int pos = 0;    // variable to store the servo position 
const int jump_interval = 30;
const int MIN_INPUT = 50;
const int MAX_INPUT = 2900;

String inString = "";

int target_degree = 1500;
int current_degree = target_degree;

int stan_servo_trans_time = 1;


void setup() { 
  
  bldc1.attach(7);  // Lower servo motor
  bldc1.writeMicroseconds(target_degree);

  Serial.begin(115200);
  Serial.print("Program Start\n");
  Serial.print("Min: 50us \t Max: 2900us \t Neu: 1700 us \n");
  Serial.print("Normal: 900 - 2100 \n");
  
} 
 
void loop() { 

  while (Serial.available() > 0){
    int inChar = Serial.read();
    if(isDigit(inChar)){
      inString += (char) inChar;
    }
    if(inChar == '\n'){
      int rev_int = inString.toInt();
      Serial.print("reveice command: ");
      Serial.println(rev_int);
      //Start turning
      if((rev_int  <  MIN_INPUT) || (rev_int  >  MAX_INPUT) ){
         Serial.print("Invalid input\n");
         inString = "";//reset the inString
      }
      else{
          target_degree = rev_int;
  
//        //Start Looping transaction
//        if(target_degree > current_degree){//increment up
//          while(target_degree > current_degree+jump_interval){
//            bldc1.writeMicroseconds(current_degree+=jump_interval);
//            delay(stan_servo_trans_time);
//            Serial.print("Current Degree: ");
//            Serial.println(current_degree);
//          }
//        }
//        else if(target_degree < current_degree){//decrement down
//          while(target_degree < current_degree-jump_interval){
//            bldc1.writeMicroseconds(current_degree-=jump_interval);
//            delay(stan_servo_trans_time);
//            Serial.print("Current Degree: ");
//            Serial.println(current_degree);
//          }
//        }
  
        //Done Transaction
        bldc1.write(target_degree);//ensure reach the target degree
        current_degree = target_degree;
        Serial.print("Current Degree: ");
        Serial.println(current_degree);
        
        inString = "";//reset the inString
        delay(500);
      }
    }
  }
    
} 

