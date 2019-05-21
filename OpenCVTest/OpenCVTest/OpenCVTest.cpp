#include <sstream>
#include "pch.h"
#include <string>
#include <iostream>
#include <thread>
#include <vector>
#include <fstream>
#include <Windows.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>

using namespace cv;
using namespace std;

vector<Mat> leftCam;
vector<Mat> rightCam;

void processVideo(VideoCapture cap, bool left) {
	Mat frame;
	cap.read(frame);

	if (left) leftCam.push_back(frame);
	else rightCam.push_back(frame);
}

void vidToPng() {
	string lis[] = { "8M_TightLeft1", "8M_Center3", "8M_TightLeft", "8M_TightLeft1", "8M_TightRight", "8M_TightRight1", "8M_TightRight2", "4M_Center", "4M_Center1" };


	vector<int> compression_params;
	compression_params.push_back(IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(0);

	for (string file : lis) {
		cout << file << endl;
		VideoCapture capL("LAXShootingFootageL/" + file + ".avi");
		int i = 1;

		while (capL.grab()) {
			cout << "  " << capL.get(CAP_PROP_POS_FRAMES) << endl;
			Mat fL;

			capL.retrieve(fL);

			if (fL.empty()) continue;
			ofstream outputFile("labels/" + file + "_frame_" + to_string(i) + ".txt");
			outputFile << endl;
			outputFile.close();
			//imwrite("Left/" + file + "_frame_" + to_string(i) + ".png", fL, compression_params);
			i++;
		}

		capL.release();

		VideoCapture capR("LAXShootingFootageR/" + file + ".avi");
		i = 1;
		while (capR.grab() && 0) {
			cout << "  " << capR.get(CAP_PROP_POS_FRAMES) << endl;
			Mat fR;

			capR.retrieve(fR);

			if (fR.empty()) continue;
			ofstream outputFile("labels/" + file + "_frame_" + to_string(i) + ".txt");
			outputFile << endl;
			outputFile.close();
			//imwrite("Right/" + file + "_frame_" + to_string(i) + ".png", fR, compression_params);
			i++;
		}

		capR.release();
	}
}

void collectFrame() {
	VideoCapture captureL(1);
	VideoCapture captureR(2);

	captureL.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	captureL.set(CV_CAP_PROP_FPS, 60);
	captureL.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	captureL.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

	captureR.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	captureR.set(CV_CAP_PROP_FPS, 60);
	captureR.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	captureR.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

	Size S = Size(1280, 720);

	TickMeter t;

	while (1) {
		string param = "";
		cin >> param;
		cout << "What video" << endl;
		cin >> param;

		string name = param.substr(0, 1) + "Meters";

		if (param[1] == '1') name += "_FarLeft";
		else if (param[1] == '2') name += "_TightLeft";
		else if (param[1] == '3') name += "_Center";
		else if (param[1] == '4') name += "_TightRight";
		else if (param[1] == '5') name += "_FarRight";

		if (param.size() > 2) name += param.substr(2, -1);

		while (1) {
			thread cameraL(processVideo, captureL, true);
			thread cameraR(processVideo, captureR, false);

			cameraL.join();
			cameraR.join();
			t.stop();
			if (t.getTimeSec() >= 1) {
				cout << "Running..." << endl;
				t.reset();
			}
			t.start();

			if (GetKeyState(VK_SPACE) & 0x8000)
				break;
		}

		VideoWriter outL("LAXShootingFootageL/" + name + ".avi", captureL.get(CV_CAP_PROP_FOURCC), captureL.get(CV_CAP_PROP_FPS), S);
		VideoWriter outR("LAXShootingFootageR/" + name + ".avi", captureR.get(CV_CAP_PROP_FOURCC), captureR.get(CV_CAP_PROP_FPS), S);

		for (Mat f : leftCam) {
			outL.write(f);
		}
		cout << "LAXShootingFootageL/" + name + ".avi complete" << endl;
		leftCam.clear();

		for (Mat f : rightCam) {
			outR.write(f);
		}
		cout << "LAXShootingFootageR/" + name + ".avi complete" << endl;
		rightCam.clear();

		outL.release();
		outR.release();
	}
}

int main() {

	vidToPng();

	return 0;
}
