// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
#include "SerialPort.h"
#include <myo/myo.hpp>
#include <ctime>                //clock
//#include <cstdio>               //clock
#include <conio.h>

#include <stdio.h>
#include <string.h>

//using namespace std;

//////////////////////////////////////////////////////////// main SerialPort arduino

char* portName = "\\\\.\\COM7";

#define MAX_DATA_LENGTH 255

char incomingData[MAX_DATA_LENGTH];

//Control signals for turning on and turning off the led
//Check arduino code
char ledON[] = "ON\n";
char ledOFF[] = "OFF\n";

//Arduino SerialPort object
SerialPort* arduino;

//Blinking Delay
const unsigned int BLINKING_DELAY = 1000;

//If you want to send data then define "SEND" else comment it out
//#define SEND 

void exampleReceiveData(void)
{
	int readResult = arduino->readSerialPort(incomingData, MAX_DATA_LENGTH);
	printf("%s", incomingData);
	Sleep(10);
}

void exampleWriteData(unsigned int delayTime)                              //simiar to serial.write - data from the myo
{

	arduino->writeSerialPort(ledON, MAX_DATA_LENGTH);
	Sleep(delayTime);
	arduino->writeSerialPort(ledOFF, MAX_DATA_LENGTH);
	Sleep(delayTime);

}

void autoConnect(void)
{
	//wait connection
	while (!arduino->isConnected()) {
		Sleep(100);
		arduino = new SerialPort(portName);
	}

	//Checking if arduino is connected or not
	if (arduino->isConnected()) {
		std::cout << "Connection established at port " << portName << std::endl;
	}

#ifdef SEND
	while (arduino->isConnected()) exampleWriteData(BLINKING_DELAY);
#else // SEND
	//	while (arduino->isConnected()) exampleReceiveData(); //this is waiting for the arduino to send data to the computer, no stop condition in the while loop
#endif // SEND

	//if the serial connection is lost
//	autoConnect();
}




//////////////////////////////////////////




// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener {
public:
	DataCollector()
		: onArm(false), isUnlocked(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose()
	{
	}

	// onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
	void onUnpair(myo::Myo* myo, uint64_t timestamp)
	{
		// We've lost a Myo.
		// Let's clean up some leftover state.
		roll_w = 0;
		pitch_w = 0;
		yaw_w = 0;
		onArm = false;
		isUnlocked = false;
	}

	// onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
	// as a unit quaternion.
	void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
	{
		using std::atan2;
		using std::asin;
		using std::sqrt;
		using std::max;
		using std::min;

		// Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
		float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
			1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
		float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
		float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
			1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));

		// Convert the floating point angles in radians to a scale from 0 to 18.   ///////////////////////////////////////////////////////
		roll_w = static_cast<float>((roll + (float)M_PI) / (M_PI * 2.0f) * 18);
		pitch_w = static_cast<float>((pitch + (float)M_PI / 2.0f) / M_PI * 18);
		yaw_w = static_cast<float>((yaw + (float)M_PI) / (M_PI * 2.0f) * 18);
		/*
		roll_w = roll;
		pitch_w = pitch;
		yaw_w = yaw;*/
	}

	// onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
	// making a fist, or not making a fist anymore.
	void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
	{
		currentPose = pose;

		if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
			// Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
			// Myo becoming locked.
			myo->unlock(myo::Myo::unlockHold);

			// Notify the Myo that the pose has resulted in an action, in this case changing
			// the text on the screen. The Myo will vibrate.
			myo->notifyUserAction();
		}
		else {
			// Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
			// are being performed, but lock after inactivity.
			myo->unlock(myo::Myo::unlockTimed);
		}
	}

	// onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
	// arm. This lets Myo know which arm it's on and which way it's facing.
	void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
		myo::WarmupState warmupState)
	{
		onArm = true;
		whichArm = arm;
	}

	// There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
	// For this example, the functions overridden above are sufficient.

	// We define this function to print the current values that were updated by the on...() functions above.
	void print()
	{
		// Clear the current line
		std::cout << '\r';

		// Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
		std::cout << '[' << std::string(roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
			<< '[' << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
			<< '[' << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << ']';

		if (onArm) {
			// Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.

			// Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
			// output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
			// that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().

			std::string poseString = currentPose.toString();                                                        //data ** current pose

			std::cout << '[' << (isUnlocked ? "unlocked" : "locked  ") << ']'
				<< '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
				<< '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
		}
		else {
			// Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
			std::cout << '[' << std::string(8, ' ') << ']' << "[?]" << '[' << std::string(14, ' ') << ']';
		}

		std::cout << std::flush;
	}

	// These values are set by onArmSync() and onArmUnsync() above.
	bool onArm;
	myo::Arm whichArm;

	// This is set by onUnlocked() and onLocked() above.
	bool isUnlocked;

	// These values are set by onOrientationData() and onPose() above.
	float roll_w, pitch_w, yaw_w;
	myo::Pose currentPose;
};

int main(int argc, char** argv)
{
	std::clock_t start;                                          //  CLOCK FUNCTION CHECK THIS!!!!
	double duration;

	start = std::clock();



	arduino = new SerialPort(portName);

	autoConnect();

	// We catch any exceptions that might occur below -- see the catch statement for more details.
	try {

		// First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
		// publishing your application. The Hub provides access to one or more Myos.
		myo::Hub hub("com.example.hello-myo");

		std::cout << "Attempting to find a Myo..." << std::endl;

		// Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
		// immediately.
		// waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
		// if that fails, the function will return a null pointer.
		myo::Myo* myo = hub.waitForMyo(10000);

		// If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
		if (!myo) {
			throw std::runtime_error("Unable to find a Myo!");
		}

		// We've found a Myo.
		std::cout << "Connected to a Myo armband!" << std::endl << std::endl;

		// Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
		DataCollector collector;

		// Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
		// Hub::run() to send events to all registered device listeners.
		hub.addListener(&collector);
		bool GripperState = true;


		// Finally we enter our main loop.
		while (1) {

			// In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
			// In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
			hub.run(1000 / 20);
			// After processing events, we call the print() member function we defined above to print out the values we've
			// obtained from any events that have occurred.

			collector.print();
			//   M1   M2   M3   M4   M5
			//char poseBuffer1[] = "A1000,2048,2048,2048,2048\n";     // FIST             "0\n";             //this values are not being read correct check this!
			//char poseBuffer2[] = "A1000,2300,2048,2048,2048\n";     // DOUBLETAP        "1\n";
			//char poseBuffer3[] = "A1000,2300,1100,2048,2048\n";     // FINGERSPREAD                 "2\n";
			//char poseBuffer4[] = "A1000,2300,1100,3100,2048\n";     // WAVEIN M4
			//char poseBuffer5[] = "A2048,1024,2048,2048,2071\n";     //WAVEOUT

			//M1: Yaw; M2: pitch; M3: roll

			float M1_yaw = collector.yaw_w;
			float M2_pitch = collector.pitch_w;
			float M3_roll = collector.roll_w;
			std::cout << " " << std::endl;
			std::cout << "M3_yaw: " << M1_yaw << "      M2_pitch: " << M2_pitch << "      M1_roll: " << M3_roll << std::endl;

			//converter				//change 123
			float yawConverter = 500;							// 3072 / 4;
			float pitchConverter = 130;				// 2048 / 18;
			float rollConverter = 130;				// 2048 / 18
			//std::cout << "Converter roll: " << rollConverter << "      pitch: " << pitchConverter << "      yaw: " << yawConverter << std::endl;

			//offset value
			int yawOffset = 1028;
			int pitchOffset = 880;
			int rollOffset = 880;
			//std::cout << "Offset roll: " << rollOffset << "      pitch: " << pitchOffset << "      yaw: " << yawOffset << std::endl;

			//calculated step Values
			float yawVal = (M1_yaw - (M1_yaw * 0.28) ) * yawConverter - yawOffset;
			float pitchVal = M2_pitch * pitchConverter+ pitchOffset;				//-to get the rotation the right way
			float rollVal = M3_roll * rollConverter + rollOffset;
			//std::cout << "yaw: " << yawVal << "      pitch: " << pitchVal << "val roll: " << rollVal << std::endl;

			//convert from float to int
			int yawValue = (int)yawVal;
			int pitchValue = (int)pitchVal;
			int rollValue = (int)rollVal;
			//std::cout << "yaw: " << yawValue << "      pitch: " << pitchValue << "      value roll: " << rollValue << std::endl;

			//convert values from float to char[]							//FUUUUUUUUUUUUUUUUUCK dad shiiiiiiit.... the char should be a char []    maybe not needed try to send int
			char BufferM1[4];
			sprintf(BufferM1, "%i", yawValue);
			char BufferM2[4];
			sprintf(BufferM2, "%i", pitchValue);
			char BufferM3[4];
			sprintf(BufferM3, "%i", rollValue);


			//sprintf(array, "%f", 3.123);
			std::cout << "BufferM1: " << BufferM1 << "      BufferM2: " << BufferM2 << "      BufferM3: " << BufferM3 << std::endl;

			//   M1   M2   M3   M4   M5
			char BufferFist[] = "2048,2048\n";						// FIST             "0\n";     open        //this values are not being read correct check this!
			char BufferFingerSpread[] = "3072,1024\n";				// DOUBLETAP        "1\n";		close
											//gripper State [true = closed]
			//char BufferM1[] = { rollChar };							//Motor1
			//char BufferM2[] = {pitchChar};						//Motor2
			//char BufferM3[] = {rollChar};							//Motor3
			char BufferA[] = "A";									//String start
			char BufferComma[] = ",";								//String interrupter



			int readResult;

			Sleep(10);
			if (collector.currentPose.type() == myo::Pose::fist || collector.currentPose.type() == myo::Pose::fingersSpread) {			//if we detect a gesture then run the cases....

				std::cout << "if ^_° : " << std::endl;

				Sleep(50);

				switch (collector.currentPose.type())			//(myo::Pose::fist)    //
				{
				case myo::Pose::fist:
					std::cout << "sending fist: " << std::endl;
					/*	char BufferFist[] = "ABufferM1,BufferM2,BufferM3,BufferG1,BufferG2\n				|| Other way
						arduino->writeSerialPort(BufferFist, "data length" );*/
					arduino->writeSerialPort(BufferA, 1);
					arduino->writeSerialPort(BufferM1, 4);
					arduino->writeSerialPort(BufferComma, 1);
					arduino->writeSerialPort(BufferM2, 4);
					arduino->writeSerialPort(BufferComma, 1);
					arduino->writeSerialPort(BufferM3, 4);
					arduino->writeSerialPort(BufferComma, 1);
					arduino->writeSerialPort(BufferFist, 11);
					GripperState = true;
					//std::cout << "Buffer_fullString: " << BufferA << "," << BufferM1 << "," << BufferM2 << "," << BufferM3 << "," << BufferFist<< "      Gripper: cloased" << std:end1
					readResult = arduino->readSerialPort(incomingData, MAX_DATA_LENGTH);       //NOT THE PROBLEM
					printf("X: %s", incomingData);
					break;
				case myo::Pose::fingersSpread:
					std::cout << "sending fingersSpread: " << std::endl;
					arduino->writeSerialPort(BufferA, 1);
					arduino->writeSerialPort(BufferM1, 4);
					arduino->writeSerialPort(BufferComma, 1);
					arduino->writeSerialPort(BufferM2, 4);
					arduino->writeSerialPort(BufferComma, 1);
					arduino->writeSerialPort(BufferM3, 4);
					arduino->writeSerialPort(BufferComma, 1);
					arduino->writeSerialPort(BufferFingerSpread, 11);
					GripperState = false;
					//std::cout << "Buffer_fullString: " << BufferA << "," << BufferM1 << "," << BufferM2 << "," << BufferM3 << "," << BufferFist<< "      Gripper: cloased" << std:end1
					readResult = arduino->readSerialPort(incomingData, MAX_DATA_LENGTH);       //NOT THE PROBLEM
					printf("X: %s", incomingData);
					break;
				}


			}
			else {				//Send Data
				if (GripperState == true) {
					std::cout << "sending fist: " << std::endl;
					arduino->writeSerialPort(BufferA, 1);
					arduino->writeSerialPort(BufferM1, 4);
					arduino->writeSerialPort(BufferComma, 1);
					arduino->writeSerialPort(BufferM2, 4);
					arduino->writeSerialPort(BufferComma, 1);
					arduino->writeSerialPort(BufferM3, 4);
					arduino->writeSerialPort(BufferComma, 1);
					arduino->writeSerialPort(BufferFist, 11);
					//std::cout << "Buffer_fullString: " << BufferA << "," << BufferM1 << "," << BufferM2 << "," << BufferM3 << "," << BufferFist<< "      Gripper: cloased" << std:end1
					readResult = arduino->readSerialPort(incomingData, MAX_DATA_LENGTH);       //NOT THE PROBLEM
					printf("X: %s", incomingData);
				}
				else {
					std::cout << "sending fist: " << std::endl;
					arduino->writeSerialPort(BufferA, 1);
					arduino->writeSerialPort(BufferM1, 4);
					arduino->writeSerialPort(BufferComma, 1);
					arduino->writeSerialPort(BufferM2, 4);
					arduino->writeSerialPort(BufferComma, 1);
					arduino->writeSerialPort(BufferM3, 4);
					arduino->writeSerialPort(BufferComma, 1);
					arduino->writeSerialPort(BufferFingerSpread, 11);
					//std::cout << "Buffer_fullString: " << BufferA << "," << BufferM1 << "," << BufferM2 << "," << BufferM3 << "," << BufferFist<< "      Gripper: cloased" << std:end1
					readResult = arduino->readSerialPort(incomingData, MAX_DATA_LENGTH);       //NOT THE PROBLEM
					printf("X: %s", incomingData);
				}
			}

			//std::cout << "VS values [ Moter1: " << BufferM1 << "; Motor2: " << BufferM2 << "; Motor3: " << BufferM3 << ; "Gripper State" << " ]" << GripperState << std::endl;

		}


		// If a standard exception occurred, we print out its message and exit.
	}
	catch (const std::exception & e) {
		std::cerr << "Error: " << e.what() << std::endl;
		std::cerr << "Press enter to continue.";
		std::cin.ignore();
		return 1;
	}
}
