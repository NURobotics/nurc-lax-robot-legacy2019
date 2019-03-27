#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/videoio/videoio.hpp>

#include "LGR/axis.hpp"
#include "LGR/HSV.hpp"
#include "LGR/camera.hpp"
#include "LGR/robot.hpp"
#include "LGR/mController.hpp"
#include "LGR/ball.hpp"

using namespace cv;
using namespace cv::cuda;
using namespace std;
using namespace LGR;

HSVvalues HSV;
Ball LAXBall;
Robot LAXBot;
mController Roboteq;

Mat ballCoordL(1,1,CV_64FC2);
Mat ballCoordR(1,1,CV_64FC2);

Mat P(4,1,CV_64F);

double frameTime = 0;

// Names that will appear at the top of each window
const string windowNameL = "Left Original Image";
const string windowNameL1 = "Left HSV Image";
const string windowNameL2 = "Left Thresholded Image";

const string windowNameR = "Right Original Image";
const string windowNameR1 = "Right HSV Image";
const string windowNameR2 = "Right Thresholded Image";
const string trackbarWindowName = "Trackbars";

void on_trackbar(int, void*) {
  HSV.calcValues();
}

void createTrackbars() {

  namedWindow(trackbarWindowName,0);

	char TrackbarName[50];
	sprintf( TrackbarName, "HMin", HSV.HMin);
	sprintf( TrackbarName, "HMax", HSV.HMax);
	sprintf( TrackbarName, "SMin", HSV.SMin);
	sprintf( TrackbarName, "SMax", HSV.SMax);
	sprintf( TrackbarName, "VMin", HSV.VMin);
	sprintf( TrackbarName, "VMax", HSV.VMax);

  createTrackbar( "HMin", trackbarWindowName, &HSV.HMin, HSV.HMax, on_trackbar );
  createTrackbar( "HMax", trackbarWindowName, &HSV.HMax, HSV.HMax, on_trackbar );
  createTrackbar( "SMin", trackbarWindowName, &HSV.SMin, HSV.SMax, on_trackbar );
  createTrackbar( "SMax", trackbarWindowName, &HSV.SMax, HSV.SMax, on_trackbar );
  createTrackbar( "VMin", trackbarWindowName, &HSV.VMin, HSV.VMax, on_trackbar );
  createTrackbar( "VMax", trackbarWindowName, &HSV.VMax, HSV.VMax, on_trackbar );
}

Mat GetCooridnates(Camera* capture, TickMeter* t, HostMem* p_l, Mat* frame) {
  frameTime += capture.FrametoHSV(t, p_l, frame);
  capture.GetThreshold(HSV.HSVMinG, HSV.HSVMaxG);
  
  return capture.FindBallPixels();
}

void MoveRobot() {
  triangulatePoints(projL, projR, ballCoordL, ballCoordR, P);
  LAXBall.nextCoords(P.at<double>(0,0)/P.at<double>(3,0),
                     P.at<double>(1,0)/P.at<double>(3,0),
                     -P.at<double>(2,0)/P.at<double>(3,0));
  frameTime /= 2;
  LAXBall.calcFinalPos(frameTime, LAXBot);
  mController.calcCounts(LAXBot);
  //mController.sendAngles();
}

int main(int argc, char* argv[]) {

  HostMem page_lockedL(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_8UC3);
  HostMem page_lockedR(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_8UC3);
  
  Mat frameL = page_lockedL;
  Mat frameR = page_lockedR;
  
  const double projLData[] = {1072.6, 0, 680.724, -194.14, 0, 1073.69, 383.223, 0, 0, 0, 1, 0};
  const double projRData[] = {1051.57, 0, 630.838, 190.334, 0, 1052.83, 341.993, 0, 0, 0, 1, 0};

  const Mat projL(3, 4, CV_64F, projLData);
  const Mat projR(3, 4, CV_64F, projRData);

  const double KLData[] = {1072.6, 0, 680.724, 0, 1073.69, 383.223, 0, 0, 1};
  const double KRData[] = {1051.57, 0, 630.838, 0, 1052.83, 341.993, 0, 0, 1};

  const Mat KL(3, 3, CV_64F, KLData);
  const Mat KR(3, 3, CV_64F, KRData);

  const double distortion_coeffsLData[] = {-0.407242, 0.163507, -0.000309831, 0.00123473, 0};
  const double distortion_coeffsRData[] = {-0.395033, 0.15755, 6.48855e-005, 0.000488116, 0};

  const Mat distortion_coeffsL(5, 1, CV_64F, distortion_coeffsLData);
  const Mat distortion_coeffsR(5, 1, CV_64F, distortion_coeffsRData);

	Camera* captureL = new Camera("Left Cam", 1, projL, KL, distortion_coeffsL);
	Camera* captureR = new Camera("Right Cam", 1, projR, KR, distortion_coeffsr);
  
  TickMeter timerL;
  TickMeter timerR;

	createTrackbars();
  namedWindow(windowNameL, WINDOW_NORMAL);
  namedWindow(windowNameL1, WINDOW_NORMAL);
  namedWindow(windowNameL2, WINDOW_NORMAL);
  namedWindow(windowNameR, WINDOW_NORMAL);
  namedWindow(windowNameR1, WINDOW_NORMAL);
  namedWindow(windowNameR2, WINDOW_NORMAL);
  
  //Roboteq.configure();
  timerL.start();
  timerR.start();

	for (;;) {
    frameTime = 0;
    
    ballCoordL = GetCooridnates(captureL, &timerL, &page_lockedL, &frameL);
    ballCoordR = GetCooridnates(captureR, &timerR, &page_lockedR, &frameR);
    if (captureL.ballFound && captureL.ballFound) {
      MoveRobot();
    }
    else {
      LAXBall.reset();
    }

	}

	return 0;
}
