#include <Arduino.h>
#include "Dynamixel_Serial.h"
#include <SoftwareSerial.h>
#include <MatrixMath.h>

//Robot defines
#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full control buffer.
#define SET_Baudrate 57600    // Baud rate speed which the Dynamixel will be set too (57600)
#define LED13 0x0D
#define CW_LIMIT_ANGLE 0x001        // lowest clockwise angle is 1, as when set to 0 it set servo to wheel mode
#define CCW_LIMIT_ANGLE 0xFFF       // Highest anit-clockwise angle is 0XFFF, as when set to 0 it set servo to wheel mode
SoftwareSerial mySerial(10, 11);    // RX, TX

#define DELAY_TIME 100
//#define led 13

//Forward things

float L_1 = 237.6;
float L_2 = 219.5;
float L_3 = 270;
float alfa_1 = 90;
float ValTheta_1;
float ValTheta_2;
float ValTheta_3;

float pi = 3.14159265359;
int motor1,motor2, motor3, motor4, motor5;
String m0, m1, m2, m3, m4, m5;

//Matrices
#define N (4)
mtx_type T0[N][N];
mtx_type T1[N][N];
mtx_type T2[N][N];
mtx_type T3[N][N];

mtx_type T_01[N][N];
mtx_type T_02[N][N];
mtx_type T_03[N][N];
mtx_type Vec_minus[3];
mtx_type Vec_1[3];

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



void loop() {
  // unsigned long currentTime = millis();  //updates frequently where we are on the millis timeline
  //TESTif ( currentTime - previousTime >= eventInterval) {
  if (Serial.available() > 0) {
    if (led) {
      digitalWrite(13, HIGH);
      led = false;
    }
    else {
      digitalWrite(13, LOW);
      led = true;
    }
    m0 = Serial.readStringUntil('A');
    m1  = Serial.readStringUntil(',');
    m2 = Serial.readStringUntil(',');
    m3 = Serial.readStringUntil(',');
    m4 = Serial.readStringUntil(',');
    m5  = Serial.readStringUntil('\n');
    int m1_Size = (int)m1.length();
    int m2_Size = (int)m2.length();
    int m3_Size = (int)m3.length();
    int m4_Size = (int)m4.length();
    int m5_Size = (int)m5.length();

    Serial.print("m1 String size: ");    Serial.println(m1_Size);
    Serial.print("m1: ");    Serial.println(m1);
    Serial.print("m2 String size: ");    Serial.println(m2_Size);
    Serial.print("m2: ");    Serial.println(m2);
    Serial.print("m3 String size: ");    Serial.println(m3_Size);
    Serial.print("m3: ");    Serial.println(m3);
    Serial.print("m4 String size: ");    Serial.println(m4_Size);
    Serial.print("m4: ");    Serial.println(m4);
    Serial.print("m5 String size: ");    Serial.println(m5_Size);
    Serial.print("m5 ");    Serial.println(m5);

    int fullString_Size = m1_Size + m2_Size + m3_Size + m4_Size + m5_Size;

    motor1 = m1.toInt();
    motor2 = m2.toInt();
    motor3 = m3.toInt();
    motor4 = m4.toInt();
    motor5 = m5.toInt();

    if (fullString_Size == 20) {                                  //checking if the string got the right size
      if (motor1 > 1000) {                                        //minimum angle of motor 1
        if (motor1 < 4096) {                                      //maximum angle of motor 1
          if (motor2 > 1000) {                                    //minimum angle of motor 2
            if (motor2 < 3200) {                                  //maximum angle of motor 2
              if (motor3 > 1000) {                                //minimum angle of motor 3
                if (motor3 < 3200) {                              //maximum angle of motor 3
                  if (motor4 > 2020) {                            //minimum angle of motor 4
                    if (motor4 < 3090) {                          //maximum angle of motor 4
                      if (motor5 > 1020) {                        //minimum angle of motor 5
                        if (motor5 < 2100) {                      //maximum angle of motor 5
                          Serial.println("-----------------Converted_values/motor1,motor2,motor3,motor4,motor5 [String] --------------------");
                          Serial.print(" motor1: ");    Serial.print(motor1);
                          Serial.print(" motor2: ");    Serial.print(motor2);
                          Serial.print(" motor3: ");    Serial.print(motor3);
                          Serial.print(" motor4: ");    Serial.print(motor4);
                          Serial.print(" motor5: ");    Serial.println(motor5);

                          Dynamixel.setNGoalPositions(motor1, motor2, motor3, motor4, motor5);
                          //TEST    previousTime = currentTime;  //update the timing for the next time around
                          //TEST}
                          //}
                          Serial.println("m1 ");
                          Serial.println(motor1); //to check if we get the same values from the motors position
                          Serial.println(motor2);
                          Serial.println(motor3);
                          Serial.println(motor4);
                          Serial.println(motor5);
                          // delay(100);

                          float val1 = 0.087890625 * motor1;
                          float val2 = 0.087890625 * motor2;
                          float val3 = 0.087890625 * motor3;
                          ForwardKinematics(val1, val2, val3);

                        }
                        else {
                          Serial.print("Angle of motor5 to big: ");
                          Serial.println(motor5);
                        }
                      }
                      else {
                        Serial.print("Angle of motor5 to tiny: ");
                        Serial.println(motor5);
                      }
                    }
                    else {
                      Serial.print("Angle of motor4 to big: ");
                      Serial.println(motor4);
                    }
                  }
                  else {
                    Serial.print("Angle of motor4 to tiny: ");
                    Serial.println(motor4);
                  }
                }
                else {
                  Serial.print("Angle of motor3 to big: ");
                  Serial.println(motor3);
                }
              }
              else {
                Serial.print("Angle of motor3 to tiny: ");
                Serial.println(motor3);
              }
            }
            else {
              Serial.print("Angle of motor2 to big: ");
              Serial.println(motor2);
            }
          }
          else {
            Serial.print("Angle of motor2 to tiny: ");
            Serial.println(motor2);
          }
        }
        else {
          Serial.print("Angle of motor1 to big: ");
          Serial.println(motor1);
        }
      }
      else {
        Serial.print("Angle of motor1 to tiny: ");
        Serial.println(motor1);
      }
    }
    else {
      Serial.println("Error: Data length got wrong size: ");
      Serial.print("String size: ");    Serial.println(fullString_Size);
    }
  }

}




float ForwardKinematics(float Theta_1, float Theta_2, float Theta_3 ) {
 Serial.print("Theta_1: ");       Serial.println(Theta_1);
 Serial.print("Theta_2: ");       Serial.println(Theta_2);
 Serial.print("Theta_3: ");       Serial.println(Theta_3);
 
  /*T0 matix
      1.0000         0         0         0
           0    1.0000         0         0
           0         0    1.0000  237.6000
           0         0         0    1.0000*/
  T0[0][0] = cos(0);            T0[0][1] = -sin(0);           T0[0][2] = 0;         T0[0][3] = 0;                   //row 1
  T0[1][0] = sin(0) * cos(0);   T0[1][1] = cos(0) * cos(0);  T0[1][2] = -sin(0);   T0[1][3] = -sin(0) * L_1;       //row 2
  T0[2][0] = sin(0) * sin(0);   T0[2][1] = cos(0) * sin(0);   T0[2][2] = cos(0);    T0[2][3] = cos(0) * L_1;        //row 3
  T0[3][0] = 0;                 T0[3][1] = 0;                 T0[3][2] = 0;         T0[3][3] = 1;
  //--------------------------------------------------------------------------
  /*T1 matix
       1     0     0     0
       0     0    -1     0
       0     1     0     0
       0     0     0     1
  */
  T1[0][0] = cos(Theta_1);              T1[0][1] = -sin(Theta_1);            T1[0][2] = 0;           T1[0][3] = 0;              //row 1
  T1[1][0] = sin(Theta_1) * cos(0);     T1[1][1] = cos(Theta_1) * cos(0);    T1[1][2] = -sin(-90);   T1[1][3] = -sin(-90) * 0;  //row 2
  T1[2][0] = sin(Theta_1) * sin(-90);   T1[2][1] = cos(Theta_1) * sin(-90);  T1[2][2] = cos(-90);    T1[2][3] = cos(-90) * 0;   //row 3
  T1[3][0] = 0;                    T1[3][1] = 0;                   T1[3][2] = 0;           T1[3][3] = 1;              //row 4
  //--------------------------------------------------------------------------
  /*T2 matix

       1     0     0   220
       0     1     0     0
       0     0     1     0
       0     0     0     1 */
  T2[0][0] = cos(Theta_2);              T2[0][1] = -sin(Theta_2);            T2[0][2] = 0;           T2[0][3] = L_2;            //row 1
  T2[1][0] = sin(Theta_2) * cos(L_2);   T2[1][1] = cos(Theta_2) * cos(L_2);  T2[1][2] = -sin(0);     T2[1][3] = -sin(0) * 0;    //row 2
  T2[2][0] = sin(Theta_2) * sin(0);     T2[2][1] = cos(Theta_2) * sin(0);    T2[2][2] = cos(0);      T2[2][3] = cos(0) * 0;     //row 3
  T2[3][0] = 0;                         T2[3][1] = 0;                        T2[3][2] = 0;           T2[3][3] = 1;              //row 4
  //--------------------------------------------------------------------------
  /*T3 matix
       1     0     0   270
       0     0     1     0
       0    -1     0     0
       0     0     0     1
  */
  T3[0][0] = cos(Theta_3);              T3[0][1] = -sin(Theta_3);            T3[0][2] = 0;          T3[0][3] = L_3;             //row 1
  T3[1][0] = sin(Theta_3) * cos(L_3);   T3[1][1] = cos(Theta_3) * cos(L_3);  T3[1][2] = -sin(90);   T3[1][3] = -sin(90) * 0;    //row 2
  T3[2][0] = sin(Theta_3) * sin(90);    T3[2][1] = cos(Theta_3) * sin(90);   T3[2][2] = cos(90);    T3[2][3] = cos(90) * 0;     //row 3
  T3[3][0] = 0;                         T3[3][1] = 0;                        T3[3][2] = 0;          T3[3][3] = 1;               //row 4
  //--------------------------------------------------------------------------

  Matrix.Multiply((mtx_type*)T0, (mtx_type*)T1, N, N, N, (mtx_type*)T_01);
  Matrix.Multiply((mtx_type*)T_01, (mtx_type*)T2, N, N, N, (mtx_type*)T_02);
  Matrix.Multiply((mtx_type*)T_02, (mtx_type*)T3, N, N, N, (mtx_type*)T_03);

  Matrix.Print((mtx_type*)T_03, N, N, "Full matrix T_03: ");
  
  Vec_minus[0] = 0;
  Vec_minus[1] = 0;
  Vec_minus[2] = 237.6;

  Vec_1[0] = T_03[0][3] - Vec_minus[0];
  Vec_1[1] = T_03[1][3] - Vec_minus[1];
  Vec_1[2] = T_03[2][3] - Vec_minus[2];

  Serial.print("X: "); Serial.println(Vec_1[0]); 
  Serial.print("Y: "); Serial.println(Vec_1[1]);
  Serial.print("Z: "); Serial.println(Vec_1[2]);
  return Vec_1[0] + Vec_1[1] + Vec_1[2];
}
