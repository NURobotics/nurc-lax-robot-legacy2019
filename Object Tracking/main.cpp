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
Ball* LAXBall = new Ball();
Robot* LAXBot = new Robot();
mController* Roboteq = new mController();

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

Mat GetCooridnates(Camera* capture, TickMeter* t, HostMem* p_l, Mat* frame) {
  
  frameTime += capture->ReadFrame(t, p_l, frame);
  
  if (capture->useHSV) {
    capture->FrameToHSV();
    capture->GetThreshold(HSV);
  }
  
  return capture->FindBallPixels();
}

void MoveRobot() {
  triangulatePoints(projL, projR, ballCoordL, ballCoordR, P);
  
  LAXBall->nextCoords(P.at<double>(0,0)/P.at<double>(3,0),
                     P.at<double>(1,0)/P.at<double>(3,0),
                     -P.at<double>(2,0)/P.at<double>(3,0));
                  
  
  LAXBall->calcFinalPos(frameTime/2, LAXBot);
  Roboteq->calcCounts(LAXBot);
  //Roboteq->sendAngles();
}

class MainEvent : public ParallelLoopBody {
  public: 
    MainEvent(Camera* captureL, TickMeter* timerL, HostMem* page_lockedL, Mat* frameL,
              Camera* captureR, TickMeter* timerR, HostMem* page_lockedR, Mat* frameR)
                : p_captureL(captureL), p_timerL(timerL), p_page_lockedL(page_lockedL), p_frameL(frameL),
                  p_captureR(captureR), p_timerR(timerR), p_page_lockedR(page_lockedR), p_frameR(frameR) {}
                
    virtual void operator ()(const Range& range) const {
      for (int r = range.start; r < range.end; r++) {
        switch (r) {
          case 0:
          ballCoordL = GetCooridnates(p_captureL, p_timerL, p_page_lockedL, p_frameL);
            break;
          case 1: ballCoordR = GetCooridnates(p_captureR, p_timerR, p_page_lockedR, p_frameR);
            break;
          case 2:
            if (p_captureL->ballFound && p_captureL->ballFound) {
              MoveRobot();
            }
            else {
              LAXBall->reset();
            }
            break;
        }
      }
    }
    
    MainEvent& operator=(const MainEvent &) {
        return *this;
    };
    
  private:
    Camera* p_captureL;
    TickMeter* p_timerL;
    HostMem* p_page_lockedL;
    Mat* p_frameL;
    Camera* p_captureR;
    TickMeter* p_timerR;
    HostMem* p_page_lockedR;
    Mat* p_frameR;
};

int main(int argc, char* argv[]) {

  HostMem page_lockedL(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_8UC4);
  HostMem page_lockedR(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_8UC4);
  
  Mat frameL = page_lockedL.createMatHeader();
  Mat frameR = page_lockedR.createMatHeader();

  double KLData[] = {1072.6, 0, 680.724, 0, 1073.69, 383.223, 0, 0, 1};
  double KRData[] = {1051.57, 0, 630.838, 0, 1052.83, 341.993, 0, 0, 1};

  const Mat KL(3, 3, CV_64F, KLData);
  const Mat KR(3, 3, CV_64F, KRData);

  double distortion_coeffsLData[] = {-0.407242, 0.163507, -0.000309831, 0.00123473, 0};
  double distortion_coeffsRData[] = {-0.395033, 0.15755, 6.48855e-005, 0.000488116, 0};

  const Mat distortion_coeffsL(5, 1, CV_64F, distortion_coeffsLData);
  const Mat distortion_coeffsR(5, 1, CV_64F, distortion_coeffsRData);

  Camera* captureL = new Camera("Left Cam", 1, projL, KL, distortion_coeffsL);
  Camera* captureR = new Camera("Right Cam", 2, projR, KR, distortion_coeffsR);
  
  TickMeter timerL;
  TickMeter timerR;
  TickMeter timerT;
  TickMeter code_timer;

  createTrackbars();
  bool showWindows = false;
  
  HSV.calcValues();
  
  MainEvent mainEvent(captureL, &timerL, &page_lockedL, &frameL,
                      captureR, &timerR, &page_lockedR, &frameR);
  
  //Roboteq.configure();
  timerL.start();
  timerR.start();

  for (;;) {
    code_timer.start();
    frameTime = 0;

    parallel_for_(Range(0, 3), mainEvent);

    if (showWindows) {      
      captureL->drawObject();
      captureR->drawObject();
      
      captureL->showOriginal();
      captureR->showOriginal();
      
      //captureL->showHSV();
      //captureR->showHSV();
      
      captureL->showThreshold();
      captureR->showThreshold();
    }
    
    cout << "FPS: " << 1/(frameTime/2) << endl;

    char c = (char) waitKey( 1 );
    if( c == 'p' )
      showWindows = !showWindows;
  }

  return 0;
}
