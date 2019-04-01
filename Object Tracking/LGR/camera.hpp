#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>

#include "constants.hpp"
#include "HSV.hpp"

using namespace std;
using namespace cv;
using namespace cv::cuda;

namespace LGR {
  struct Camera {
    string name;
    int feed;
    
    VideoCapture cap;
    Mat Proj;
    Mat K;
    Mat distortion_coeffs;
    
    Mat currFrameInt;
    GpuMat currFrame;
    
    GpuMat HSV;
    
    GpuMat threshold;
    Mat outputThreshold;
    
    bool ballFound = false;
    double ballPixelX;
    double ballPixelY;
    
    double FrametoHSV(TickMeter* t, HostMem* p_l, Mat* frame);
    void GetThreshold(GpuMat HSVMin, GpuMat HSVMax);
    Mat FindBallPixels();
    Mat findBall();
    Mat trackBall();
    
    void drawObject();
    
    void showOriginal();
    void showHSV();
    void showThreshold();
    
    void print();
        
    Camera(string n, int f, Mat p, Mat k, Mat d);
    
    const Mat erodeElement = getStructuringElement(MORPH_RECT,Size(6,6));
    const Mat dilateElement = getStructuringElement(MORPH_RECT,Size(16,16));
    
    const Ptr<Filter> erodeFilter = cuda::createMorphologyFilter(MORPH_ERODE, CV_8UC1, erodeElement);
    const Ptr<Filter> dialateFilter = cuda::createMorphologyFilter(MORPH_DILATE, CV_8UC1, dilateElement);
  };
  
  Camera::Camera(string n, int f, Mat p, Mat k, Mat d) {
    name = n;
    feed = f;
    Proj = p;
    K = k;
    distortion_coeffs = d;
    
    cap.open(feed);
    
    cap.set(CV_CAP_PROP_FPS, FRAMES_PER_SECOND);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
  }
  
  double Camera::FrametoHSV(TickMeter* t, HostMem* p_l, Mat* frame) {
    t->stop();
    double secs = t->getTimeSec();
    t->reset();
    t->start(); 
    cap.read(currFrameInt);
    
    currFrame.upload(currFrameInt);
    cuda::cvtColor(currFrame, HSV, COLOR_BGR2HSV, 4);
    
    return secs;
  }
  
  void Camera::GetThreshold(HSVvalues values) {
    GpuMat HSVchannels[4];
    GpuMat thresholdMaxchannels[3];
    GpuMat thresholdMinchannels[3];
    GpuMat thresholdchannels[4];
    
    cuda::split(HSV, HSVchannels);
    
    cuda::threshold(HSVchannels[0], thresholdMinchannels[0], values.Hmin, 255, THRESH_BINARY_INV);
    cuda::threshold(HSVchannels[0], thresholdMaxchannels[0], values.Hmax, 255, THRESH_BINARY);
    cuda::bitwise_and(thresholdMaxchannels[0], thresholdMinchannels[0], thresholdchannels[0]);
    
    cuda::threshold(HSVchannels[1], thresholdMinchannels[1], values.Smin, 255, THRESH_BINARY_INV);
    cuda::threshold(HSVchannels[1], thresholdMaxchannels[1], values.Smax, 255, THRESH_BINARY);
    cuda::bitwise_and(thresholdMaxchannels[1], thresholdMinchannels[1], thresholdchannels[1]);
    
    cuda::threshold(HSVchannels[2], thresholdMinchannels[2], values.Vmin, 255, THRESH_BINARY_INV);
    cuda::threshold(HSVchannels[2], thresholdMaxchannels[2], values.Vmax, 255, THRESH_BINARY);
    cuda::bitwise_and(thresholdMaxchannels[2], thresholdMinchannels[2], thresholdchannels[2]);
    
    cuda::bitwise_and(thresholdchannels[0], thresholdchannels[1], thresholdchannels[3]);
    cuda::bitwise_and(thresholdchannels[3], thresholdchannels[2], threshold);
    
    erodeFilter->apply(threshold, threshold);
    dialateFilter->apply(threshold, threshold);
    
    threshold.download(outputThreshold);
  }
  
  //This is where the money gets made!
  Mat Camera::findBall() {
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Mat undistoredCoord;
    Mat ballCoord(1, 1, CV_64FC2);
    ballPixelX = 0;
    ballPixelY = 0;
    
    findContours(outputThreshold, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    // Uses moments method to find our filtered object
    double refArea = 0;
    if (hierarchy.size() > 0) {
      int numObjects = hierarchy.size();

      if (numObjects < MAX_NUM_OBJECTS) {
        for (int i = 0; i >= 0; i = hierarchy[i][0]) {

	        Moments moment = moments((cv::Mat)contours[i]);
	        double area = moment.m00;
	
	        if (area > MIN_OBJECT_AREA && area < MAX_OBJECT_AREA && area > refArea) {
		        ballPixelX = moment.m10/area;
		        ballPixelY = moment.m01/area;
		        ballFound = true;
		        refArea = area;
	        } 
	        else {
	          ballFound = false;
	        }
        }
      }
    }
    
    ballCoord.at<Vec2d>(0,0)[0] = ballPixelX;
    ballCoord.at<Vec2d>(0,0)[1] = ballPixelY;
    
    if (ballFound) {
      undistortPoints(ballCoord, undistoredCoord, K, distortion_coeffs);
    }
    
    return ballCoord;
  }
  
  
  Mat Camera::trackBall() {
    return findBall();
  }
  
  Mat Camera::FindBallPixels() {
    if (ballFound) {
      return trackBall();
    }
    else {
      return findBall();
    }
  }
  
  void Camera::drawObject() {
    if (ballFound) {
      circle(currFrameInt, cvPoint(ballPixelX, ballPixelY), 20, cvScalar(0, 255, 0), 2);
      if (ballPixelY - 25 > 0)
        line(currFrameInt,  Point(ballPixelX, ballPixelY), Point(ballPixelX, ballPixelY-25), Scalar(0, 255, 0), 2);
      else
        line(currFrameInt, Point(ballPixelX, ballPixelY), Point(ballPixelX, 0), Scalar(0, 255, 0), 2);
      if (ballPixelY + 25 < FRAME_HEIGHT)
        line(currFrameInt, Point(ballPixelX, ballPixelY), Point(ballPixelX, ballPixelY+25), Scalar(0, 255, 0), 2);
      else
        line(currFrameInt, Point(ballPixelX, ballPixelY), Point(ballPixelX,  FRAME_HEIGHT), Scalar(0, 255, 0), 2);
      if (ballPixelX - 25 > 0)
        line(currFrameInt, Point(ballPixelX, ballPixelY), Point(ballPixelX-25, ballPixelY), Scalar(0, 255, 0), 2);
      else
        line(currFrameInt, Point(ballPixelX, ballPixelY), Point(0, ballPixelY), Scalar(0, 255, 0), 2);
      if (ballPixelX + 25 < FRAME_WIDTH)
        line(currFrameInt, Point(ballPixelX, ballPixelY), Point(ballPixelX+25, ballPixelY), Scalar(0, 255, 0), 2);
      else 
        line(currFrameInt, Point(ballPixelX, ballPixelY), Point(FRAME_WIDTH, ballPixelY), Scalar(0, 255, 0), 2);

      putText(currFrameInt, to_string(ballPixelX) + ", " + to_string(ballPixelY), Point(ballPixelX, ballPixelY+30), 1, 1, Scalar(0, 255, 0), 2);
    }
  }
  
  void Camera::showOriginal() {
    imshow(name + " Original Image", currFrameInt);
  }
  
  void Camera::showHSV() {
    Mat f;
    HSV.download(f);
    imshow(name + " HSV Image", f);
  }
  
  void Camera::showThreshold() {
    imshow(name + " Threshold Image", outputThreshold);
  }
  
  void Camera::print() {
    cout << name << ": Feed " << feed << endl;
    if (ballFound) {
      cout << "Tracking Ball" << endl;
    }
    else {
      cout << "Finding Ball" << endl;
    }
  }
}
