/*
 Motor Test

 Just see if the robot can move and turn.

 Circuit:
 * Arduino Robot

 created 1 May 2013
 by X. Yang
 modified 12 May 2013
 by D. Cuartielles

 This example is in the public domain
 */

#include <ArduinoRobot.h>
#include <Wire.h>
#include <SPI.h>

const int Gauche = M4;
const int Droit = M0;
const int avantGauche = M3;
const int avantDroit = M1;

void setup() {
  // initialize the robot
  Robot.begin();
  Robot.beginTFT();
  Serial.begin(9600);
}

void loop() {
  
  Robot.motorsWrite(255, 255);  // move forward
  delay(2000);
  Robot.motorsWrite(0, 0);      // slow stop
  delay(1500);
}

long pingPinFunction(int pingPin){
  long duration, inches, cm;
  
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  
  
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  
  cm = microsecondsToCentimeters(duration);
  
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  
  return cm;
  }
  
  void pinAction(){
      
    //cas mur Gauche et en face
    if(pingPinFunction(Gauche)<=15 && pingPinFunction(avantGauche)<=15){
         Robot.motorsWrite(0, 0);
        Robot.motorsWrite(225, 255);
         Robot.turn(90);
      }else if(pingPinFunction(Droit)<=15 && pingPinFunction(avantDroit)<=15){
         Robot.motorsWrite(0, 0);
        Robot.motorsWrite(225, 255);
         Robot.turn(-90);
      }else if(pingPinFunction(Gauche)<=5){
       // Robot.motorsWrite(0, 0);
        //Robot.motorsWrite(225, 255);
         //Robot.turn(25);
      }else if(pingPinFunction(Droit)<=5){
        //Robot.motorsWrite(0, 0);
        //Robot.motorsWrite(225, 255);
        // Robot.turn(-25);
      }else{
        Robot.motorsWrite(255, 255);
      }
      
  }

long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}
