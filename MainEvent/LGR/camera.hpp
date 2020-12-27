#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <string>
#include <vector>

#include "../ML/yolo_v2_class.hpp"
#include "constants.hpp"
// Kuba put these in here
#include <cstring>
#include <opencv2/core/utility.hpp> // is this the same as core core?
#include <opencv2/tracking.hpp>

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
  Mat cameraMatrix;
  Mat distortionCoeffs;

  Mat currFrame; // current frame you're looking at (current image) (image)
  GpuMat currFrameGPU;

  GpuMat HSV;

  GpuMat threshold;
  Mat outputThreshold;

  string modelConfig = "../ML/yolov3_tiny.cfg";
  string modelWeights = "../ML/yolov3_tiny.weights";

  // Using OpenCV DNN
  vector<string> classes;
  Net net;
  string outputLayer;

  // Using Darknet DNN
  Detector *detector = new Detector(modelConfig, modelWeights);
  vector<bbox_t> result_vec;

  bool ballFound = false;
  bool useHSV = false;

  double ReadFrame(TickMeter &time);
  void FrameToHSV();
  void GetThreshold(HSVvalues values);
  Mat FindballPositions();
  Mat findBallHSV();
  vector<Mat> findBallML();
  Mat trackBall();

  void drawObject();

  void showOriginal();
  void showHSV();
  void showThreshold();

  void print();

  Camera(string n, int f, Mat c, Mat d);

  const Mat erodeElement = getStructuringElement(MORPH_RECT, Size(6, 6));
  const Mat dilateElement = getStructuringElement(MORPH_RECT, Size(16, 16));

  const Ptr<Filter> erodeFilter =
      cuda::createMorphologyFilter(MORPH_ERODE, CV_8UC1, erodeElement);
  const Ptr<Filter> dialateFilter =
      cuda::createMorphologyFilter(MORPH_DILATE, CV_8UC1, dilateElement);

  // Kuba put this stuff here
  // This looks wrong, because we're going to be tracking multiple objects
  // TODO: Examine whether this needs to be a multitracker
  Ptr<Tracker> tracker =
      TrackerKCF::create(); // need to research this tracker_algorithm
};

Camera::Camera(string n, int f, Mat c, Mat d)
    : name(n), feed(f), cameraMatrix(c), distortionCoeffs(d) {
  // cap.open("../VID_0.avi");
  cap.open("/dev/video" + to_string(feed), CAP_V4L2);

  cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
  cap.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
  cap.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

  // classes.push_back("laxball");
  // net = readNetFromDarknet(modelConfig, modelWeights);
  // net.setPreferableBackend(DNN_BACKEND_OPENCV);
  // net.setPreferableTarget(DNN_TARGET_OPENCL);

  // vector<int> outLayers = net.getUnconnectedOutLayers();
  // vector<String> layersNames = net.getLayerNames();

  // outputLayer = layersNames[outLayers[0]];
}

double Camera::ReadFrame(TickMeter &time) {
  time.stop();
  double secs = time.getTimeSec();
  time.reset();
  time.start();

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

  cuda::threshold(HSVchannels[0], thresholdMinchannels[0], values.HMin, 255,
                  THRESH_BINARY);
  cuda::threshold(HSVchannels[0], thresholdMaxchannels[0], values.HMax, 255,
                  THRESH_BINARY_INV);
  cuda::bitwise_and(thresholdMinchannels[0], thresholdMaxchannels[0],
                    thresholdchannels[0]);

  cuda::threshold(HSVchannels[1], thresholdMinchannels[1], values.SMin, 255,
                  THRESH_BINARY);
  cuda::threshold(HSVchannels[1], thresholdMaxchannels[1], values.SMax, 255,
                  THRESH_BINARY_INV);
  cuda::bitwise_and(thresholdMinchannels[1], thresholdMaxchannels[1],
                    thresholdchannels[1]);

  cuda::threshold(HSVchannels[2], thresholdMinchannels[2], values.VMin, 255,
                  THRESH_BINARY);
  cuda::threshold(HSVchannels[2], thresholdMaxchannels[2], values.VMax, 255,
                  THRESH_BINARY_INV);
  cuda::bitwise_and(thresholdMinchannels[2], thresholdMaxchannels[2],
                    thresholdchannels[2]);

  cuda::bitwise_and(thresholdchannels[0], thresholdchannels[1],
                    thresholdchannels[3]);
  cuda::bitwise_and(thresholdchannels[3], thresholdchannels[2], threshold);

  erodeFilter->apply(threshold, threshold);
  dialateFilter->apply(threshold, threshold);

  threshold.download(outputThreshold);
}

// This is where the money gets made!
Mat Camera::findBallHSV() {
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  Mat undistoredCoord;
  Mat ballCoord(1, 1, CV_64FC2);

  findContours(outputThreshold, contours, hierarchy, RETR_CCOMP,
               CHAIN_APPROX_SIMPLE);

  // Uses moments method to find our filtered object
  double refArea = 0;
  if (hierarchy.size() > 0) {
    int numObjects = hierarchy.size();

    if (numObjects < MAX_NUM_OBJECTS) {
      for (int i = 0; i >= 0; i = hierarchy[i][0]) {
        Moments moment = moments((cv::Mat)contours[i]);
        double area = moment.m00;

        if (area > MIN_OBJECT_AREA && area < MAX_OBJECT_AREA &&
            area > refArea) {
          ballBox.x = moment.m10 / area;
          ballBox.y = moment.m01 / area;
          ballFound = true;
          refArea = area;
          double diameter = 2 * sqrt(area / 3.14);
          cout << "ball diameter is " << diameter << endl;
        } else {
          ballFound = false;
        }
      }
    }
  }

  ballCoord.at<Vec2d>(0, 0)[0] = ballBox.x;
  ballCoord.at<Vec2d>(0, 0)[1] = ballBox.y;

  if (ballFound) {
    undistortPoints(ballCoord, undistoredCoord, cameraMatrix, distortionCoeffs);
  }

  return ballCoord;
}

vector<Mat> Camera::findBallML() {
  vector<Mat> ballCoords;
  result_vec = detector->detect(currFrame, CONF_THRESHOLD);
  result_vec = detector->tracking_id(
      result_vec); // this is important for updating the boundingbox

  if ((ballFound = result_vec.size() > 0)) {
    for (auto result : result_vec) {
      Mat ballCoord(1, 1, CV_64FC2);
      ballCoord.at<Vec2d>(0, 0)[0] = result.x;
      ballCoord.at<Vec2d>(0, 0)[1] = result.y;
      ballCoords.push_back(ballCoord);
    }
  }

  // COMPLETED: Change ballCoord to a vector;
  return ballCoords; // ball's coordinates
}

Mat Camera::trackBall() { tracker->update(frame, ballBox); }

vector<Mat> Camera::FindballPositions() {
  vector<Mat> ballCoords;
  if (ballFound) {
    ballCoords = trackBall();
  } else {
    if (useHSV) {
      ballCoords.push_back(findBallHSV());
    } else {
      ballCoords = findBallML();
    }
    if (ballFound) {
      tracker->init(currFrame, ballBox) // initiates the ball tracker
    }
  }
  return ballCoords;
}

void Camera::drawObject() {
  Scalar color(60, 160, 260);
  for (auto &i : result_vec) {
    rectangle(currFrame,
              Rect(i.x, i.y, i.w, i.h),
              color,
              3);
    putText(currFrame,
            "laxball",
            Point2f(i.x, i.y - 10),
            FONT_HERSHEY_COMPLEX_SMALL,
            1,
            color);
    putText(currFrame,
            to_string(i.prob),
            cv::Point2f(i.x + 5, i.y + 35),
            FONT_HERSHEY_COMPLEX_SMALL,
            1,
            color);
    if (i.track_id > 0) {
      putText(currFrame, 
              to_string(i.track_id),
              cv::Point2f(i.x + 15, i.y + 15), 
              FONT_HERSHEY_COMPLEX_SMALL,
              1,
              color);
    }
  }
}

void Camera::showOriginal() {
  imshow(name + " Original Image", currFrame);
}

void Camera::showHSV() {
  Mat image;
  HSV.download(image);
  imshow(name + " HSV Image", image);
}

void Camera::showThreshold() {
  imshow(name + " Threshold Image", outputThreshold);
}

void Camera::print() {
  cout << name << ": Feed " << feed << endl;
  if (ballFound) {
    cout << "Tracking Ball" << endl;
  } else {
    cout << "Finding Ball" << endl;
  }
}
} // namespace LGR
