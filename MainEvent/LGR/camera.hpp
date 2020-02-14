#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>

#include "constants.hpp"

#include "../ML/yolo_v2_class.hpp"
//Kuba put these in here
#include <opencv2/core/utility.hpp>  // is this the same as core core?
#include <opencv2/tracking.hpp>
#include <cstring>


using namespace std;
using namespace cv;
    // classes.push_back("laxball");
    // net = readNetFromDarknet(modelConfig, modelWeights);
    // net.setPreferableBackend(DNN_BACKEND_OPENCV);
    // net.setPreferableTarget(DNN_TARGET_OPENCL);
using namespace cv::cuda;
using namespace cv::dnn;

namespace LGR {
  struct Camera {
    string name;
    int feed;

    VideoCapture cap;
    Mat Proj;
    Mat K;
    Mat distortion_coeffs;

    Mat currFrame; //current frame you're looking at (current image) (image)
    GpuMat currFrameGPU;

    GpuMat HSV;

    GpuMat threshold;
    Mat outputThreshold;

    string modelConfig = "../ML/yolov3_tiny.cfg";
    string modelWeights = "../ML/yolov3_tiny.weights";

    //Using OpenCV DNN
    vector<string> classes;
    Net net;
    string outputLayer;

    //Using Darknet DNN
    Detector *detector = new Detector(modelConfig, modelWeights);
    vector<bbox_t> result_vec;

    bool ballFound = false;
    bool useHSV = false;
    double ballBox.x;
    double ballBox.y;
    double ballBox.w;
    double ballBox.h:

    double ReadFrame(TickMeter& t);
    void FrameToHSV();
    void GetThreshold(HSVvalues values);
    Mat FindballPositions();
    Mat findBallHSV();
    Mat findBallML();
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


    //Kuba put this stuff here
    static Rect2d ballBox;
    Ptr<Tracker> tracker = TrackerKCF::create(); //need to research this tracker_algorithm
  };






  Camera::Camera(string n, int f, Mat p, Mat k, Mat d) {
    name = n;
    feed = f;
    Proj = p;
    K = k;
    distortion_coeffs = d;

    cap.open("../VID_0.avi");
    //cap.open("/dev/video" + to_string(f), CAP_V4L2);

    //cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    //cap.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    //cap.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);


    // classes.push_back("laxball");
    // net = readNetFromDarknet(modelConfig, modelWeights);
    // net.setPreferableBackend(DNN_BACKEND_OPENCV);
    // net.setPreferableTarget(DNN_TARGET_OPENCL);

    vector<int> outLayers = net.getUnconnectedOutLayers();
    // vector<String> layersNames = net.getLayerNames();

    //outputLayer = layersNames[outLayers[0]];
  }

  double Camera::ReadFrame(TickMeter& t) {
    t.stop();
    double secs = t.getTimeSec();
    t.reset();
    t.start();
    cout << secs << endl;
    cap.read(currFrame);


    return secs;
  }

  void Camera::FrameToHSV() {
    currFrameGPU.upload(currFrame);
    cuda::cvtColor(currFrameGPU, HSV, COLOR_BGR2HSV, 4);
  }

  void Camera::GetThreshold(HSVvalues values) {

    GpuMat HSVchannels[4];
    GpuMat thresholdMaxchannels[3];
    GpuMat thresholdMinchannels[3];
    GpuMat thresholdchannels[4];

    cuda::split(HSV, HSVchannels);

    cuda::threshold(HSVchannels[0], thresholdMinchannels[0], values.HMin, 255, THRESH_BINARY);
    cuda::threshold(HSVchannels[0], thresholdMaxchannels[0], values.HMax, 255, THRESH_BINARY_INV);
    cuda::bitwise_and(thresholdMinchannels[0], thresholdMaxchannels[0], thresholdchannels[0]);

    cuda::threshold(HSVchannels[1], thresholdMinchannels[1], values.SMin, 255, THRESH_BINARY);
    cuda::threshold(HSVchannels[1], thresholdMaxchannels[1], values.SMax, 255, THRESH_BINARY_INV);
    cuda::bitwise_and(thresholdMinchannels[1], thresholdMaxchannels[1], thresholdchannels[1]);

    cuda::threshold(HSVchannels[2], thresholdMinchannels[2], values.VMin, 255, THRESH_BINARY);
    cuda::threshold(HSVchannels[2], thresholdMaxchannels[2], values.VMax, 255, THRESH_BINARY_INV);
    cuda::bitwise_and(thresholdMinchannels[2], thresholdMaxchannels[2], thresholdchannels[2]);

    cuda::bitwise_and(thresholdchannels[0], thresholdchannels[1], thresholdchannels[3]);
    cuda::bitwise_and(thresholdchannels[3], thresholdchannels[2], threshold);

    erodeFilter->apply(threshold, threshold);
    dialateFilter->apply(threshold, threshold);

    threshold.download(outputThreshold);
  }

  //This is where the money gets made!
  Mat Camera::findBallHSV() {
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Mat undistoredCoord;
    Mat ballCoord(1, 1, CV_64FC2);

    findContours(outputThreshold, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

    // Uses moments method to find our filtered object
    double refArea = 0;
    if (hierarchy.size() > 0) {
      int numObjects = hierarchy.size();

      if (numObjects < MAX_NUM_OBJECTS) {
        for (int i = 0; i >= 0; i = hierarchy[i][0]) {

	        Moments moment = moments((cv::Mat)contours[i]);
	        double area = moment.m00;

	        if (area > MIN_OBJECT_AREA && area < MAX_OBJECT_AREA && area > refArea) {
		        ballBox.x = moment.m10/area;
		        ballBox.y = moment.m01/area;
		        ballFound = true;
		        refArea = area;
		        double diameter = 2 * sqrt(area / 3.14);
		        cout << "ball diameter is " << diameter << endl;
	        }
	        else {
	          ballFound = false;
	        }
        }
      }
    }

    ballCoord.at<Vec2d>(0,0)[0] = ballBox.x;
    ballCoord.at<Vec2d>(0,0)[1] = ballBox.y;

    if (ballFound) {
      undistortPoints(ballCoord, undistoredCoord, K, distortion_coeffs);
    }

    return ballCoord;
  }

  Mat Camera::findBallML() {
    Mat ballCoord(1, 1, CV_64FC2);

  	result_vec = detector->detect(currFrame, CONF_THRESHOLD);
    result_vec = detector->tracking_id(result_vec); //this is important for updating the boundingbox

    if ((ballFound = result_vec.size() > 0)) {
      ballBox = result_vec[0];
      ballCoord.at<Vec2d>(0,0)[0] = ballBox.x;
      ballCoord.at<Vec2d>(0,0)[1] = ballBox.y; 
    }

    return ballCoord; //ball's coordinates
  }

  // TO DO if needed
  Mat Camera::trackBall() {
    tracker->update(frame,ballBox);
  }

  Mat Camera::FindballPositions() {
    Mat returnMe;
    if (ballFound) {
      returnMe = trackBall();
    }
    else {
      if (useHSV) {
        returnMe =  findBallHSV();
      }
      else{
        returnMe = findBallML();
      }
      if (ballFound) {
        tracker->init(currFrame, ballBox) //initiates the ball tracker
      }
    }
    return returnMe;
  }

  void Camera::drawObject() {
    if (ballFound) {
      if (useHSV)
        circle(currFrame, Point(ballBox.x, ballBox.y), 20, Scalar(0, 255, 0), 2);
      if (ballBox.y - 25 > 0)
        line(currFrame,  Point(ballBox.x, ballBox.y), Point(ballBox.x, ballBox.y-25), Scalar(0, 255, 0), 2);
      else
        line(currFrame, Point(ballBox.x, ballBox.y), Point(ballBox.x, 0), Scalar(0, 255, 0), 2);
      if (ballBox.y + 25 < FRAME_HEIGHT)
        line(currFrame, Point(ballBox.x, ballBox.y), Point(ballBox.x, ballBox.y+25), Scalar(0, 255, 0), 2);
      else
        line(currFrame, Point(ballBox.x, ballBox.y), Point(ballBox.x,  FRAME_HEIGHT), Scalar(0, 255, 0), 2);
      if (ballBox.x - 25 > 0)
        line(currFrame, Point(ballBox.x, ballBox.y), Point(ballBox.x-25, ballBox.y), Scalar(0, 255, 0), 2);
      else
        line(currFrame, Point(ballBox.x, ballBox.y), Point(0, ballBox.y), Scalar(0, 255, 0), 2);
      if (ballBox.x + 25 < FRAME_WIDTH)
        line(currFrame, Point(ballBox.x, ballBox.y), Point(ballBox.x+25, ballBox.y), Scalar(0, 255, 0), 2);
      else
        line(currFrame, Point(ballBox.x, ballBox.y), Point(FRAME_WIDTH, ballBox.y), Scalar(0, 255, 0), 2);

      if (useHSV)
        putText(currFrame, to_string(ballBox.x) + ", " + to_string(ballBox.y), Point(ballBox.x, ballBox.y+30), 1, 1, Scalar(0, 255, 0), 2);
    }

    if (!useHSV) {
      for (auto &i : result_vec) {
        Scalar color(60, 160, 260);
        rectangle(currFrame, Rect(i.x, i.y, i.w, i.h), color, 3);
        putText(currFrame, "laxball", Point2f(i.x, i.y - 10), FONT_HERSHEY_COMPLEX_SMALL, 1, color);
        putText(currFrame, to_string(i.prob), cv::Point2f(i.x+5, i.y + 35), FONT_HERSHEY_COMPLEX_SMALL, 1, color);
        if(i.track_id > 0)
          putText(currFrame, to_string(i.track_id), cv::Point2f(i.x+15, i.y + 15), FONT_HERSHEY_COMPLEX_SMALL, 1, color);
      }
    }
  }

  void Camera::showOriginal() {
    imshow(name + " Original Image", currFrame);
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
