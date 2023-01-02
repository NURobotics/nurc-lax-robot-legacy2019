#include <iostream>
// #include <stdio.h>
// #include <string.h>
#include <string.h>

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

using namespace std;

int main(int argc, char* argv[]) {

	string response = "";
	//Create an instance of RoboteqDevice.
	RoboteqDevice device;
	//Connect to the device, for windows use "\\\\.\\com1" for com1.
	//int status = device.Connect("/dev/ttyS0");
	int status = device.Connect("\\\\.\\com3");

	//Check to see if the connection succeeded.

	if (status != RQ_SUCCESS) {
		cout << "Error connecting to device: " << status << "." << endl;
		return 1;
	}


	cout << "- SetConfig(_DINA, 1, 1)...";
	if ((status = device.SetConfig(_DINA, 1, 1)) != RQ_SUCCESS) {
		cout << "failed --> " << status << endl;
	}
	else {
		cout << "succeeded." << endl;
	}

	//Wait 10 ms before sending another command to device
	sleepms(10);
	int result;
	cout << "- GetConfig(_DINA, 1)...";

	if ((status = device.GetConfig(_DINA, 1, result)) != RQ_SUCCESS) {
		cout << "failed --> " << status << endl;
	}
	else {
		cout << "returned --> " << result << endl;
	}

	//Wait 10 ms before sending another command to device
	sleepms(10);


	cout << "- GetValue(_ANAIN, 1)...";

	if ((status = device.GetValue(_ANAIN, 1, result)) != RQ_SUCCESS) {
		cout << "failed --> " << status << endl;
	}
	else {
		cout << "returned --> " << result << endl;
	}

	//Wait 10 ms before sending another command to device
	sleepms(10);
	cout << "- SetCommand(_GO, 1, 1)...";
	if ((status = device.SetCommand(_GO, 1, 1)) != RQ_SUCCESS) {
		cout << "failed --> " << status << endl;
	}
	else {
		cout << "succeeded." << endl;
	}



	//main loop, for detecting commands
	while (1) {

		//detect user input


		//option A: send commands to the motors
		//setConfig
	

		//ACCEL

		//DECEL


		//DIGITAL OUT

		//ESTOP


		//GO: SET MOTOR1 COMMAND








		//setCommand



		//option B: get current data from the motors
		//getConfig
		//getValue










	}











	device.Disconnect();
	return 0;
}