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

using namespace std;
using namespace cv;
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

    Mat currFrame;
    GpuMat currFrameGPU;

    GpuMat HSV;

    GpuMat threshold;
    Mat outputThreshold;

    string modelConfig = "../ML/yolov3.cfg";
    string modelWeights = "../ML/yolov3.weights";

    //Using OpenCV DNN
    vector<string> classes;
    Net net;
    string outputLayer;

    //Using Darknet DNN
    Detector *detector = new Detector(modelConfig, modelWeights);
    vector<bbox_t> result_vec;

    bool ballFound = false;
    bool useHSV = false;
    double ballPixelX;
    double ballPixelY;

    double ReadFrame(TickMeter* t);
    void FrameToHSV();
    void GetThreshold(HSVvalues values);
    Mat FindBallPixels();
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
  };

  Camera::Camera(string n, int f, Mat p, Mat k, Mat d) {
    name = n;
    feed = f;
    Proj = p;
    K = k;
    distortion_coeffs = d;

    cap.open("../TestVid.mp4");

    // cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    // cap.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    // cap.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    //
    // classes.push_back("laxball");
    // net = readNetFromDarknet(modelConfig, modelWeights);
    // net.setPreferableBackend(DNN_BACKEND_OPENCV);
    // net.setPreferableTarget(DNN_TARGET_OPENCL);
    //
    // vector<int> outLayers = net.getUnconnectedOutLayers();
    // // vector<String> layersNames = net.getLayerNames();
    //
    // outputLayer = layersNames[outLayers[0]];


  }

  double Camera::ReadFrame(TickMeter* t) {
    t->stop();
    double secs = t->getTimeSec();
    t->reset();
    t->start();
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
		        ballPixelX = moment.m10/area;
		        ballPixelY = moment.m01/area;
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

    ballCoord.at<Vec2d>(0,0)[0] = ballPixelX;
    ballCoord.at<Vec2d>(0,0)[1] = ballPixelY;

    if (ballFound) {
      undistortPoints(ballCoord, undistoredCoord, K, distortion_coeffs);
    }

    return ballCoord;
  }

  /* Old Machine Learning code, uses OpenCV DNN

  Mat Camera::findBallML() {
    Mat ballCoord(1, 1, CV_64FC2);
    Mat blob = blobFromImage(currFrame, 1/255.0, Size(NET_SIZE, NET_SIZE), Scalar(0,0,0), true, false);
    net.setInput(blob);

    Mat outs;
    net.forward(outs, outputLayer);

    float* data = (float*)outs.data;
    vector<float> confidences;
    vector<Rect> boxes;

    int rows = outs.rows;
    int cols = outs.cols;
    for (int j = 0; j < rows; ++j, data += cols) {
      Mat scores = outs.row(j).colRange(5, cols);
      double confidence;

      // Get the value and location of the maximum score
      cuda::minMaxLoc(scores, 0, &confidence, 0, 0);
      if (confidence > CONF_THRESHOLD) {
        int centerX = (int)(data[0] * FRAME_WIDTH);
        int centerY = (int)(data[1] * FRAME_HEIGHT);
        int width = (int)(data[2] * FRAME_WIDTH);
        int height = (int)(data[3] * FRAME_HEIGHT);
        int left = centerX - width / 2;
        int top = centerY - height / 2;

        confidences.push_back((float)confidence);
        boxes.push_back(Rect2d(left, top, width, height));
      }
    }

    vector<int> indices;
    NMSBoxes(boxes, confidences, CONF_THRESHOLD, NMS_THRESHOLD, indices);

    ballCoord.at<Vec2d>(0,0)[0] = boxes[indices[0]].x;
    ballCoord.at<Vec2d>(0,0)[1] = boxes[indices[0]].y;

    return ballCoord;
  }
  */

  Mat Camera::findBallML() {
    Mat ballCoord(1, 1, CV_64FC2);

  	result_vec = detector->detect(currFrame);

    if ((ballFound = result_vec.size() > 0)) {
      ballCoord.at<Vec2d>(0,0)[0] = result_vec[0].x;
      ballPixelX = result_vec[0].x;
      ballCoord.at<Vec2d>(0,0)[1] = result_vec[0].y;
      ballPixelY = result_vec[0].y;
      for (auto &i : result_vec) {
        cout << "obj_id = " << i.obj_id << ",  x = " << i.x << ", y = " << i.y
      		<< ", w = " << i.w << ", h = " << i.h
      		<< std::setprecision(3) << ", prob = " << i.prob << endl;
      }
    }

    return ballCoord;
  }

  // TO DO if needed
  Mat Camera::trackBall() {
    return findBallML();
  }

  Mat Camera::FindBallPixels() {
    if (ballFound) {
      return trackBall();
    }
    else {
      if (useHSV) {
        return findBallHSV();
      }
      else {
        return findBallML();
      }
    }
  }

  void Camera::drawObject() {
    if (ballFound) {
      circle(currFrame, Point(ballPixelX, ballPixelY), 20, Scalar(0, 255, 0), 2);
      if (ballPixelY - 25 > 0)
        line(currFrame,  Point(ballPixelX, ballPixelY), Point(ballPixelX, ballPixelY-25), Scalar(0, 255, 0), 2);
      else
        line(currFrame, Point(ballPixelX, ballPixelY), Point(ballPixelX, 0), Scalar(0, 255, 0), 2);
      if (ballPixelY + 25 < FRAME_HEIGHT)
        line(currFrame, Point(ballPixelX, ballPixelY), Point(ballPixelX, ballPixelY+25), Scalar(0, 255, 0), 2);
      else
        line(currFrame, Point(ballPixelX, ballPixelY), Point(ballPixelX,  FRAME_HEIGHT), Scalar(0, 255, 0), 2);
      if (ballPixelX - 25 > 0)
        line(currFrame, Point(ballPixelX, ballPixelY), Point(ballPixelX-25, ballPixelY), Scalar(0, 255, 0), 2);
      else
        line(currFrame, Point(ballPixelX, ballPixelY), Point(0, ballPixelY), Scalar(0, 255, 0), 2);
      if (ballPixelX + 25 < FRAME_WIDTH)
        line(currFrame, Point(ballPixelX, ballPixelY), Point(ballPixelX+25, ballPixelY), Scalar(0, 255, 0), 2);
      else
        line(currFrame, Point(ballPixelX, ballPixelY), Point(FRAME_WIDTH, ballPixelY), Scalar(0, 255, 0), 2);

      putText(currFrame, to_string(ballPixelX) + ", " + to_string(ballPixelY), Point(ballPixelX, ballPixelY+30), 1, 1, Scalar(0, 255, 0), 2);
    }
    else if (!useHSV) {
      for (auto &i : result_vec) {
        Scalar color(60, 160, 260);
        rectangle(currFrame, Rect(i.x, i.y, i.w, i.h), color, 3);
        if(obj_names.size() > i.obj_id)
          putText(currFrame, "laxball", Point2f(i.x, i.y - 10), FONT_HERSHEY_COMPLEX_SMALL, 1, color);
        if(i.track_id > 0)
          putText(currFrame, to_string(i.track_id), cv::Point2f(i.x+5, i.y + 15), FONT_HERSHEY_COMPLEX_SMALL, 1, color);
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
