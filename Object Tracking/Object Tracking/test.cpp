#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>

using namespace std;
using namespace cv;

int main() {
  VideoCapture cap("/dev/video1", CAP_V4L2);
  cap.set(CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
  Mat frame;
  
  TickMeter t;
  TickMeter f;
  t.start(); 
  f.start(); 
  
  for (;;) {
    cap.read(frame);
    t.stop();
    f.stop();
    
    if (f.getTimeSec() >= 1) {
      cout << 1/t.getTimeSec() << endl;
      f.reset();
    }
    
    t.reset();
    t.start(); 
    f.start(); 
    
    imshow("window", frame);
    
    waitKey(1);
  }
  
  return 0;
}
