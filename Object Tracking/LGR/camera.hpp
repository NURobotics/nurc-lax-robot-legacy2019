#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>

#include "constants.hpp"

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
    GpuMat HSVMinResult;
    GpuMat HSVMaxResult;
    GpuMat HSVResult;
    
    GpuMat threshold;
    Mat outputThreshold;
    
    bool ballFound = false;
    
    double FrametoHSV(TickMeter* t, HostMem* p_l, Mat* frame);
    void GetThreshold(GpuMat HSVMin, GpuMat HSVMax);
    Mat FindBallPixels();
    Mat findBall();
    Mat trackBall();
    
    void print();
        
    Camera(string n, int f, Mat p, Mat k, Mat d);
    
    const Mat erodeElement = getStructuringElement(MORPH_RECT,Size(6,6));
    const Mat dilateElement = getStructuringElement(MORPH_RECT,Size(16,16));
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
    t->start();
    cap.read(frame);
    
    frame.reshape(4);
    
    currFrame.upload(p_l);
    cuda::cvtColor(currFrame, HSV, COLOR_BGR2HSV);
    
    return secs;
  }
  
  void Camera::GetThreshold(GpuMat HSVMin, GpuMat HSVMax) {
    cuda::compare(HSV, HSVMin, HSVMinResult, CMP_GE);
    cuda::compare(HSV, HSVMax, HSVMaxResult, CMP_LE);
    
    cuda::max(HSVMinResult, HSVMaxResult, HSVResult);
    
    cuda::threshold(HSVResult, threshold, 255, 255, THRESH_BINARY);
    
    cuda::createMorphologyFilter(MORPH_ERODE, CV_8UC4, erodeElement);
    cuda::createMorphologyFilter(MORPH_DILATE, CV_8UC4, dilateElement);
    
    threshold.download(outputThreshold);
  }
  
  //This is where the money gets made!
  Mat Camera::findBall() {
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Mat undistoredCoord;
    Mat ballCoord(1, 1, CV_64FC2);
    ballCoord.at<Vec2d>(0,0)[0] = 0;
    ballCoord.at<Vec2d>(0,0)[1] = 0;

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
		        ballCoord.at<Vec2d>(0,0)[0] = moment.m10/area;
		        ballCoord.at<Vec2d>(0,0)[1] = moment.m01/area;
		        ballFound = true;
		        refArea = area;
	        } 
	        else {
	          ballFound = false;
	        }
        }
      }
    }
    
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
