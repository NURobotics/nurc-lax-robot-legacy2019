#include <string>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>

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
Ball LAXBall = Ball();
Robot LAXBot = Robot();
mController Roboteq = mController();

double projLData[] = {1072.6, 0, 680.724, -194.14, 0, 1073.69, 383.223, 0, 0, 0, 1, 0};
double projRData[] = {1051.57, 0, 630.838, 190.334, 0, 1052.83, 341.993, 0, 0, 0, 1, 0};

const Mat projL(3, 4, CV_64F, projLData);
const Mat projR(3, 4, CV_64F, projRData);

Mat ballCoordL(1,1,CV_64FC2);
Mat ballCoordR(1,1,CV_64FC2);

Mat P(4,1,CV_64F);

double frameTime = 0;

void on_trackbar(int, void*) {
  //HSV.calcValues();
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

  createTrackbar( "HMin", trackbarWindowName, &HSV.HMin, HSV.HMax, on_trackbar );
  createTrackbar( "HMax", trackbarWindowName, &HSV.HMax, HSV.HMax, on_trackbar );
  createTrackbar( "SMin", trackbarWindowName, &HSV.SMin, HSV.SMax, on_trackbar );
  createTrackbar( "SMax", trackbarWindowName, &HSV.SMax, HSV.SMax, on_trackbar );
  createTrackbar( "VMin", trackbarWindowName, &HSV.VMin, HSV.VMax, on_trackbar );
  createTrackbar( "VMax", trackbarWindowName, &HSV.VMax, HSV.VMax, on_trackbar );
}

Mat GetCooridnates(Camera& capture, TickMeter& t) {

  frameTime += capture.ReadFrame(t);

  if (capture.useHSV) {
    capture.FrameToHSV();
    capture.GetThreshold(HSV);
  }

  return capture.FindBallPixels();
}

void MoveRobot() {
  triangulatePoints(projL, projR, ballCoordL, ballCoordR, P);

  LAXBall.nextCoords(P.at<double>(0,0)/P.at<double>(3,0),
                     P.at<double>(1,0)/P.at<double>(3,0),
                     -P.at<double>(2,0)/P.at<double>(3,0));

  LAXBall.calcFinalPos(frameTime/2, LAXBot);
  Roboteq.calcCounts(LAXBot);
  //Roboteq.sendAngles();
}

class MainEvent : public ParallelLoopBody {
  public:
    MainEvent(Camera& captureL, TickMeter& timerL,
              Camera& captureR, TickMeter& timerR)
                : p_captureL(captureL), p_timerL(timerL),
                  p_captureR(captureR), p_timerR(timerR) {}

    virtual void operator ()(const Range& range) const {
      for (int r = range.start; r < range.end; r++) {
        switch (r) {
          case 0: ballCoordL = GetCooridnates(p_captureL, p_timerL);
            break;
          case 1: ballCoordR = GetCooridnates(p_captureR, p_timerR);
            break;
          case 2:
            if (p_captureL.ballFound && p_captureL.ballFound) {
              MoveRobot();
            }
            else {
              LAXBall.reset();
            }
            break;
        }
      }
    }

    MainEvent& operator=(const MainEvent &) {
        return *this;
    };

  private:
    Camera& p_captureL;
    TickMeter& p_timerL;
    Camera& p_captureR;
    TickMeter& p_timerR;
};

int main(int argc, char* argv[]) {
  double KLData[] = {1072.6, 0, 680.724, 0, 1073.69, 383.223, 0, 0, 1};
  double KRData[] = {1051.57, 0, 630.838, 0, 1052.83, 341.993, 0, 0, 1};

  const Mat KL(3, 3, CV_64F, KLData);
  const Mat KR(3, 3, CV_64F, KRData);

  double distortion_coeffsLData[] = {-0.407242, 0.163507, -0.000309831, 0.00123473, 0};
  double distortion_coeffsRData[] = {-0.395033, 0.15755, 6.48855e-005, 0.000488116, 0};

  const Mat distortion_coeffsL(5, 1, CV_64F, distortion_coeffsLData);
  const Mat distortion_coeffsR(5, 1, CV_64F, distortion_coeffsRData);

  Camera captureL = Camera("Left Cam", 1, projL, KL, distortion_coeffsL);
  Camera captureR = Camera("Right Cam", 2, projR, KR, distortion_coeffsR);

  TickMeter timerL;
  TickMeter timerR;
  TickMeter timerT;

  createTrackbars();
  bool showWindows = true;

  HSV.calcValues();

  MainEvent mainEvent(captureL, timerL,
                      captureR, timerR);

  //Roboteq.configure();
  timerL.start();
  timerR.start();

  double frameTimeTotal = 0;
  int iterations = 0;

  VideoWriter outVid("ResultL.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 60, Size(FRAME_WIDTH, FRAME_HEIGHT));

  for (;;) {
    frameTime = 0;

    parallel_for_(Range(0, 3), mainEvent);

    if (showWindows) {
      captureL.drawObject();
      //captureR.drawObject();

      captureL.showOriginal();
      //captureR.showOriginal();

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

    cout << "FPS: " << 1/((frameTimeTotal/iterations)/2)<< endl;
    for (auto &i : captureL.result_vec) {
      cout << "obj_id = " << i.obj_id << ",  x = " << i.x << ", y = " << i.y
        << ", w = " << i.w << ", h = " << i.h << ", track_id = " << (i.track_id ? i.track_id : -1)
        << std::setprecision(3) << ", prob = " << i.prob << endl;
    }

    char c = (char) waitKey( 1 );
    if( c == 'p' )
      showWindows = !showWindows;
  }

  return 0;
}
