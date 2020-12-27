#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stack>
#include <string>

#include "LGR/HSV.hpp"
#include "LGR/axis.hpp"
#include "LGR/ball.hpp"
#include "LGR/camera.hpp"
#include "LGR/mController.hpp"
#include "LGR/robot.hpp"

using namespace cv;
using namespace cv::cuda;
using namespace std;
using namespace LGR;

HSVvalues HSV;
Ball LAXBall = Ball();
Robot LAXBot = Robot();
mController Roboteq = mController();

double projLData[] = {1072.6,  0, 680.724, -194.14, 0, 1073.69,
                      383.223, 0, 0,       0,       1, 0};
double projRData[] = {1051.57, 0, 630.838, 190.334, 0, 1052.83,
                      341.993, 0, 0,       0,       1, 0};

const Mat projL(3, 4, CV_64F, projLData);
const Mat projR(3, 4, CV_64F, projRData);

// DONE: Change ballCoord to a vector
// DONE: Create a ballCoord vector that will holds the pairs of matching left
// and right balls
vector<Mat> ballCoordL; // balls recognized from left camera
vector<Mat> ballCoordR; // balls recognized from right camera
vector<pair<Mat, Mat>> pairedBalls;
vector<Ball> triangulatedBalls;
// holds balls after sorted
// ancillary code Mat ballCoordL(1,1,CV_64FC2);
// ancillary code Mat ballCoordR(1,1,CV_64FC2);

// TODO (Kuba): Find the values
// these are the bounds of the camera visions.
const double leftBound;
const double rightBound;
// margin of error for Y axis
const int marginOfError;

double frameTime = 0;

void on_trackbar(int, void *) {
  // HSV.calcValues();
}

void createTrackbars() {
  string trackbarWindowName = "HSV Trackbars";
  namedWindow(trackbarWindowName, 0);

  char TrackbarName[50];
  sprintf(TrackbarName, "HMin");
  sprintf(TrackbarName, "HMax");
  sprintf(TrackbarName, "SMin");
  sprintf(TrackbarName, "SMax");
  sprintf(TrackbarName, "VMin");
  sprintf(TrackbarName, "VMax");

  createTrackbar("HMin", trackbarWindowName, &HSV.HMin, HSV.HMax, on_trackbar);
  createTrackbar("HMax", trackbarWindowName, &HSV.HMax, HSV.HMax, on_trackbar);
  createTrackbar("SMin", trackbarWindowName, &HSV.SMin, HSV.SMax, on_trackbar);
  createTrackbar("SMax", trackbarWindowName, &HSV.SMax, HSV.SMax, on_trackbar);
  createTrackbar("VMin", trackbarWindowName, &HSV.VMin, HSV.VMax, on_trackbar);
  createTrackbar("VMax", trackbarWindowName, &HSV.VMax, HSV.VMax, on_trackbar);
}

Mat GetCooridnates(Camera &capture, TickMeter &t) {

  frameTime += capture.ReadFrame(t);

  if (capture.useHSV) {
    capture.FrameToHSV();
    capture.GetThreshold(HSV);
  }

  return capture.FindBallPixels();
}

void MoveRobot() {
  // DONE: Line 77, 79, and 83 into a function over multiple balls in the vector
  // of pairs, then check which one will arrive soonest and send that into
  // calcCounts
  // TODO (Kuba): Talk to Albert about how this functions. I think we forgot to
  // add the 'choosing closest ball part'
  for (auto ballPair : pairedBalls) {
    Ball new_ball = Ball();
    Mat xyzHomogeneous(4, 1, CV_64F);
    Mat xyz(3, 1, CV_64F);

    triangulatePoints(projL, projR, ballPair.first, ballCoord.second,
                      xyzHomogeneous);
    convertPointsFromHomogeneous(xyzHomogeneous, xyz);

    new_ball.storeNewCoordinate(xyz.at<double>(0, 0), xyz.at<double>(1, 0),
                                -xyz.at<double>(2, 0));
    new_ball.calcFinalPos(frameTime / 2);
    triangulatedBalls.push_back(new_ball)
    // treats pair[0] as left ball and pair[1] as right ball
  }

  Roboteq.calcCounts(LAXBot);
  // Roboteq.sendAngles();
}

bool sortingMethod(Mat &a, Mat &b) { return a.y > b.y; }

//TODO: Review this code. The break thing looks weird.
void matchingMethod() {
  // so these are sorted right now
  sort(ballCoordR.begin(), ballCoordR.end(), sortingMethod());
  sort(ballCoordL.begin(), ballCoordL.end(), sortingMethod());

  stack<Mat> correctedR;
  stack<Mat> correctedL;

  for (auto rball : ballCoordR) {
    if (rball.X < rightBound) {
      correctedR.push(rball);
    }
  }
  for (auto lball : ballCoordL) {
    if (lball.X > leftBound) {
      correctedL.push(lball);
    }
  }

  while (!correctedR.empty()) {
    Mat rball = correctedR.pop();
    while (!correctedL.empty()) {
      Mat lball = correctedL.top();

      if (abs(rball.y - lball.y) < marginOfError) {
        lball = correctedL.pop();
        pair<Mat, Mat> newPair;
        newPair.first = lball;
        newPair.second = rball;
        pairedBalls.push_back(newPair);
        break;
      }
      // rball.y > ball.y means that ball.y wont fit with any rball
      else if (rball.y > lball.y)
        correctedL.pop();
      // if rball.y < ball.y means that rball wont fit with any lball
      else
        break;
    }
  }
}

class MainEvent : public ParallelLoopBody {
public:
  MainEvent(Camera &captureL, TickMeter &timerL, Camera &captureR,
            TickMeter &timerR)
      : p_captureL(captureL), p_timerL(timerL), p_captureR(captureR),
        p_timerR(timerR) {}

  virtual void operator()(const Range &range) const {
    for (int thread = range.start; thread < range.end; thread++) {
      switch (thread) {
      case 0:
        ballCoordL = GetCooridnates(p_captureL, p_timerL); // change this
        break;
      case 1:
        ballCoordR = GetCooridnates(p_captureR, p_timerR); // change this
        break;
      case 2:
        if (p_captureL.ballFound && p_captureL.ballFound) {
          matchingMethod();
          MoveRobot();
        } else {
          LAXBall.reset();
        }
        break;
      }
    }
  }

  MainEvent &operator=(const MainEvent &) { return *this; };

private:
  Camera &p_captureL;
  TickMeter &p_timerL;
  Camera &p_captureR;
  TickMeter &p_timerR;
};

int main(int argc, char *argv[]) {
  double cameraMatrixLData[] = {1072.6,  0, 680.724, 0, 1073.69,
                                383.223, 0, 0,       1};
  double cameraMatrixRData[] = {1051.57, 0, 630.838, 0, 1052.83,
                                341.993, 0, 0,       1};

  const Mat cameraMatrixL(3, 3, CV_64F, cameraMatrixLData);
  const Mat cameraMatrixR(3, 3, CV_64F, cameraMatrixRData);

  double distortionCoeffsLData[] = {-0.407242, 0.163507, -0.000309831,
                                    0.00123473, 0};
  double distortionCoeffsRData[] = {-0.395033, 0.15755, 6.48855e-005,
                                    0.000488116, 0};

  const Mat distortionCoeffsL(5, 1, CV_64F, distortionCoeffsLData);
  const Mat distortionCoeffsR(5, 1, CV_64F, distortionCoeffsRData);

  Camera captureL = Camera("Left Cam", 0, cameraMatrixL, distortionCoeffsL);
  Camera captureR = Camera("Right Cam", 2, cameraMatrixR, distortionCoeffsR);

  TickMeter timerL;
  TickMeter timerR;

  createTrackbars();
  bool showWindows = true;

  HSV.calcValues();

  MainEvent mainEvent(captureL, timerL, captureR, timerR);

  // Roboteq.configure();
  timerL.start();
  timerR.start();

  double frameTimeTotal = 0;
  int iterations = 0;

  VideoWriter outVid("Result0.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30,
                     Size(FRAME_WIDTH, FRAME_HEIGHT));

  for (;;) {
    frameTime = 0;

    parallel_for_(Range(0, 3), mainEvent);

    if (showWindows) {
      captureL.drawObject();
      captureR.drawObject();

      captureL.showOriginal();
      captureR.showOriginal();

      if (captureL.useHSV) {
        captureL.showHSV();
        captureR.showHSV();

        captureL.showThreshold();
        captureR.showThreshold();
      }

      outVid << captureL.currFrame;
    }

    frameTimeTotal = frameTimeTotal + frameTime;
    iterations++;

    cout << "FPS: " << 1 / ((frameTimeTotal / iterations) / 2) << endl;
    for (auto &i : captureL.result_vec) {
      cout << "obj_id = " << i.obj_id << ",  x = " << i.x << ", y = " << i.y
           << ", w = " << i.w << ", h = " << i.h
           << ", track_id = " << (i.track_id ? i.track_id : -1)
           << std::setprecision(3) << ", prob = " << i.prob << endl;
    }

    char c = (char)waitKey(1);
    if (c == 'p')
      showWindows = !showWindows;
  }

  return 0;
}
