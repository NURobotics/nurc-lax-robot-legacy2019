#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>

using namespace cv;
using namespace std;

vector<Mat> leftCam;
vector<Mat> rightCam;
bool recording = false;
int vidCount = 5; 
string name = "VID_" + to_string(vidCount);

const Size S = Size(1280, 720);

void processVideo(VideoCapture& cap, bool left) {
	Mat frame;
	cap.read(frame);

	if (left) leftCam.push_back(frame);
	else rightCam.push_back(frame);
}

void saveVideo(vector<Mat> Cam, bool left) {
  if (Cam.empty()) return;
  string side = "L";
  
  if (left) leftCam.clear();
  else {
    rightCam.clear();
    side = "R";
  }
  
  string vidName = "/media/nvidia/USB32/OCT_23/Footage" + side + "/" + name + ".avi";
  VideoWriter out(vidName, VideoWriter::fourcc('M', 'J', 'P', 'G'), 60, S);

  for (Mat f : Cam) {
	  out.write(f);
  }
  
  cout << vidName + " complete" << endl;
  if (left) {
		vidCount++;
    name = "VID_" + to_string(vidCount);
  }
  Cam.clear();
}

void vidToPng() {
	string lis[] = { "8M_TightLeft1", "8M_Center3", "8M_TightLeft", "8M_TightLeft1", "8M_TightRight", "8M_TightRight1", "8M_TightRight2", "4M_Center", "4M_Center1" };


	vector<int> compression_params;
	compression_params.push_back(IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(0);

	for (string file : lis) {
		cout << file << endl;
		VideoCapture capL("../LAXShootingFootageL/" + file + ".avi");
		int i = 1;

		while (capL.grab()) {
			cout << "  " << capL.get(CAP_PROP_POS_FRAMES) << endl;
			Mat fL;

			capL.retrieve(fL);

			if (fL.empty()) continue;
			ofstream outputFile("labels/" + file + "_frame_" + to_string(i) + ".txt");
			outputFile << endl;
			outputFile.close();
			//imwrite("Left/" + file + "_frame_" + to_string(i) + ".png", fL, compression_params);
			i++;
		}

		capL.release();

		VideoCapture capR("LAXShootingFootageR/" + file + ".avi");
		i = 1;
		while (capR.grab() && 0) {
			cout << "  " << capR.get(CAP_PROP_POS_FRAMES) << endl;
			Mat fR;

			capR.retrieve(fR);

			if (fR.empty()) continue;
			ofstream outputFile("labels/" + file + "_frame_" + to_string(i) + ".txt");
			outputFile << endl;
			outputFile.close();
			//imwrite("Right/" + file + "_frame_" + to_string(i) + ".png", fR, compression_params);
			i++;
		}

		capR.release();
	}
}

class MainEvent : public ParallelLoopBody {
  public:
    MainEvent(VideoCapture& captureL, VideoCapture& captureR)
                : p_captureL(captureL), p_captureR(captureR) {}

    virtual void operator ()(const Range& range) const {
      for (int r = range.start; r < range.end; r++) {
        switch (r) {
          case 0:
             if (recording) {
              processVideo(p_captureL, true);
             }
            break;
          case 1:
             if (recording) {
              processVideo(p_captureR, false);
             }
            break;
          case 2:
            if (!recording) {
              saveVideo(leftCam, true);
            }
            break;
          case 3:
            if (!recording) {
              saveVideo(rightCam, false);
            }
            break;
        }
      }
    }

    MainEvent& operator=(const MainEvent &) {
        return *this;
    };

  private:
    VideoCapture& p_captureL;
    VideoCapture& p_captureR;
};

void collectFrame() {
	VideoCapture captureL("/dev/video" + to_string(2), CAP_V4L2);
	VideoCapture captureR("/dev/video" + to_string(1), CAP_V4L2);

	captureL.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
	captureL.set(CAP_PROP_FPS, 60);
	captureL.set(CAP_PROP_FRAME_WIDTH, 1280);
	captureL.set(CAP_PROP_FRAME_HEIGHT, 720);

	captureR.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
	captureR.set(CAP_PROP_FPS, 60);
	captureR.set(CAP_PROP_FRAME_WIDTH, 1280);
	captureR.set(CAP_PROP_FRAME_HEIGHT, 720);
	
	string name = "OCT_23_VID_" + to_string(vidCount);
	MainEvent mainEvent(captureL, captureR);

	TickMeter t;
  cout << "Video Ready" << endl;
  Mat placeHolder = Mat::zeros(1, 1, CV_8S);

	while (1) {
		parallel_for_(Range(0, 4), mainEvent);
		imshow("s", placeHolder);
		
		char letter = waitKey(1);
		if (letter == ' ') {
		  if (recording) cout << "Switching to saving" << endl;
		  else cout << "Switching to recording" << endl;
		  
		  recording = !recording;
		}
	}
}

void showCamera() {
	VideoCapture captureL("/dev/video" + to_string(2), CAP_V4L2);
	VideoCapture captureR("/dev/video" + to_string(1), CAP_V4L2);

	captureL.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
	captureL.set(CAP_PROP_FPS, 60);
	captureL.set(CAP_PROP_FRAME_WIDTH, 1280);
	captureL.set(CAP_PROP_FRAME_HEIGHT, 720);

	captureR.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
	captureR.set(CAP_PROP_FPS, 60);
	captureR.set(CAP_PROP_FRAME_WIDTH, 1280);
	captureR.set(CAP_PROP_FRAME_HEIGHT, 720);
  
  while(1) {
    Mat left, right;
    captureL.read(left);
    captureR.read(right);
    
    imshow("Left", left); imshow("Right", right);
    waitKey(1);
  }
}

int main() {

  collectFrame();
  
	return 0;
}
