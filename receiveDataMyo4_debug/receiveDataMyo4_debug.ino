#include <Printers.h>

/*
   Author: Manash Kumar Mandal
   Example For receiving data from SerialPort
*/
#include <Arduino.h>
#include "Dynamixel_Serial.h"
#include <SoftwareSerial.h>


//Robot defines
#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full control buffer.
#define SET_Baudrate 57600    // Baud rate speed which the Dynamixel will be set too (57600)
#define LED13 0x0D
#define CW_LIMIT_ANGLE 0x001        // lowest clockwise angle is 1, as when set to 0 it set servo to wheel mode
#define CCW_LIMIT_ANGLE 0xFFF       // Highest anit-clockwise angle is 0XFFF, as when set to 0 it set servo to wheel mode
SoftwareSerial mySerial(10, 11);    // RX, TX

#define DELAY_TIME 100
//#define led 13

int motor1;
int motor2;
int motor3;
int motor4;
int motor5;

const unsigned long eventInterval = 1000;  //constant because the event should not change
unsigned long previousTime = 0;     //variable to update the time  - unsigned long because the value of millis gets really large


String receivedString;
boolean led = true;

void setup() {

  Serial.flush();
  pinMode(led, OUTPUT);
  Serial.begin(SET_Baudrate);

  pinMode(13, OUTPUT);

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
  Dynamixel.setProfileVelocity(0x01, 100);      //Set the Profile Velocity for each servo. (max. is 1023)
  Dynamixel.setProfileVelocity(0x02, 100);
  Dynamixel.setProfileVelocity(0x03, 100);
  Dynamixel.setProfileVelocity(0x04, 100);
  Dynamixel.setProfileVelocity(0x05, 100);
  Serial.println("Done");

  Serial.print("setHoldingTorque: ");
  Dynamixel.setHoldingTorque(0x01, true);       //Turn on hold torque on servo 1
  Dynamixel.setHoldingTorque(0x02, true);
  Dynamixel.setHoldingTorque(0x03, true);
  Dynamixel.setHoldingTorque(0x04, true);
  Dynamixel.setHoldingTorque(0x05, true);
  Serial.println("Done");



  Serial.println("Setup complete.");
  Serial.println("");

  ///////////////////////////////////////////////////////////////////////

  motor1 = 2048;
  motor2 = 2048;
  motor3 = 2048;
  motor4 = 2048;
  motor5 = 2048;
  Dynamixel.setNGoalPositions(motor1, motor2, motor3, motor4, motor5);


  Serial.println("-----------------StartPossition--------------------");
  Serial.print(" motor1: ");    Serial.print(motor1);
  Serial.print(" motor2: ");    Serial.print(motor2);
  Serial.print(" motor3: ");    Serial.print(motor3);
  Serial.print(" motor4: ");    Serial.print(motor4);
  Serial.print(" motor5: ");    Serial.println(motor5);


}

String m0, m1, m2, m3, m4, m5;


void loop() {

  // unsigned long currentTime = millis();  //updates frequently where we are on the millis timeline

  //TESTif ( currentTime - previousTime >= eventInterval) {

  if (Serial.available() > 0) {
    //  String m0;

    m0 = Serial.readStringUntil('A');

    if (led) {
      digitalWrite(13, HIGH);
      led = false;
    }
    else {
      digitalWrite(13, LOW);
      led = true;
    }

    m1  = Serial.readStringUntil(',');

    m2 = Serial.readStringUntil(',');

    m3 = Serial.readStringUntil(',');

    m4 = Serial.readStringUntil(',');

    m5  = Serial.readStringUntil('\n');

    motor1 = m1.toInt();
    motor2 = m2.toInt();
    motor3 = m3.toInt();
    motor4 = m4.toInt();
    motor5 = m5.toInt();

    Serial.println("-----------------Converted_values/motor1,motor2,motor3,motor4,motor5 [String] --------------------");
    Serial.print(" motor1: ");    Serial.print(motor1);
    Serial.print(" motor2: ");    Serial.print(motor2);
    Serial.print(" motor3: ");    Serial.print(motor3);
    Serial.print(" motor4: ");    Serial.print(motor4);
    Serial.print(" motor5: ");    Serial.println(motor5);
    /*
      motor1 = m1.toInt();
      motor2 = m2.toInt();
      motor3 = m3.toInt();
      motor4 = m4.toInt();
      motor5 = m5.toInt();

      Dynamixel.setNGoalPositions(motor1, motor2, motor3, motor4, motor5);

      //TEST    previousTime = currentTime;  //update the timing for the next time around
      //TEST}
      }
      /* Serial.println("m1 ");
      Serial.println(motor1); //to check if we get the same values from the motors position
      Serial.println(motor2);
      Serial.println(motor3);
      Serial.println(motor4);
      Serial.println(motor5); */


    // delay(100);
  }
}
