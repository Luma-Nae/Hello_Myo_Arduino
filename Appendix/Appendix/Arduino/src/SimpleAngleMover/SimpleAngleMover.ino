//------------------Code to test the angles----------------
#include <Arduino.h>
#include "Dynamixel_Serial.h"
#include <SoftwareSerial.h>
#include <math.h>

//Robot defines
#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full control buffer.
#define SET_Baudrate 57600    // Baud rate speed which the Dynamixel will be set too (57600)
#define LED13 0x0D
#define CW_LIMIT_ANGLE 0x001        // lowest clockwise angle is 1, as when set to 0 it set servo to wheel mode
#define CCW_LIMIT_ANGLE 0xFFF       // Highest anit-clockwise angle is 0XFFF, as when set to 0 it set servo to wheel mode
SoftwareSerial mySerial(10, 11);    // RX, TX

//double deg2steps= 360/4096;

int motor1;
int motor2;
int motor3;
int motor4;
int motor5;

//change those angles
double a = 360-123.3407;
double b = 3.8357;
double c = 33.1805;

//plot Your angles here.
double Theta_1 = a;
double Theta_2 = b + 90;
double Theta_3 = c + 180;



void setup() {
  Serial.flush();
  Serial.begin(SET_Baudrate);

  Serial.print("starting Dynamixle communication: ");
  mySerial.begin(SET_Baudrate);  
  Dynamixel.begin(mySerial);
  Dynamixel.setDirectionPin(SERVO_ControlPin);
  Serial.println("Done");

  Serial.print("setProfileAcceleration: ");
  Dynamixel.setProfileAcceleration(0x01, 10);  //Set the Profile Acceleration for each servo. (max. is 32767)
  Dynamixel.setProfileAcceleration(0x02, 10);
  Dynamixel.setProfileAcceleration(0x03, 10);
  Dynamixel.setProfileAcceleration(0x04, 300);
  Dynamixel.setProfileAcceleration(0x05, 300);
  Serial.println("Done");

  Serial.print("setProfileVelocity: ");
  Dynamixel.setProfileVelocity(0x01, 10);      //Set the Profile Velocity for each servo. (max. is 1023)
  Dynamixel.setProfileVelocity(0x02, 10);
  Dynamixel.setProfileVelocity(0x03, 10);
  Dynamixel.setProfileVelocity(0x04, 10);
  Dynamixel.setProfileVelocity(0x05, 10);
  Serial.println("Done");

  Serial.print("setHoldingTorque: ");
  Dynamixel.setHoldingTorque(0x01, true);       //Turn on hold torque on servo 1
  Dynamixel.setHoldingTorque(0x02, true);
  Dynamixel.setHoldingTorque(0x03, true);
  Dynamixel.setHoldingTorque(0x04, true);
  Dynamixel.setHoldingTorque(0x05, true);
  Serial.println("Done");

  Serial.print("Go to start possition: ");
  Dynamixel.setNGoalPositions(2048, 2048, 2048, 2048, 2048);
  delay(3000);
  Serial.println("Done");

  Serial.print("Calculating angles: ");
  motor1 = Theta_1 * 4096/360;//deg2steps;
  motor2 = Theta_2 * 4096/360;//deg2steps;
  motor3 = Theta_3 * 4096/360;//deg2steps;
  Serial.println("Done");
  
  Serial.println("Setup completed!!!    (|^_^|)");
  Serial.println("");
}

void loop() {
Serial.println("--------------------Move 2 position--------------------");
  Serial.print("Theta_1: ");        Serial.print(Theta_1);
  Serial.print("         Motor_step_Possition_1: ");        Serial.println(motor1);
  Serial.print("Theta_2: ");        Serial.print(Theta_2);
  Serial.print("         Motor_step_Possition_2: ");        Serial.println(motor2);
  Serial.print("Theta_3: ");        Serial.print(Theta_3);
  Serial.print("         Motor_step_Possition_3: ");        Serial.println(motor3);
  Dynamixel.setNGoalPositions(motor1, motor2, motor3, 1940, 2160);
 unsigned int Load = Dynamixel.readLoad(0x01);
  Serial.print("| ");
  Serial.print(counter);
  Serial.print(" | ");
  Serial.print(Load);
  Serial.print(" |");
  delay(3000);
  Serial.println("-------------------------Done--------------------------");

  while (true){
    //nothing, so it just runs the code ones :)
  }

}
