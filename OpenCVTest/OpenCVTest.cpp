#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <thread>
#include <queue>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;
const Size S = Size(1280, 720);
/*
queue<Mat> leftCam;
queue<Mat> rightCam;

queue<queue<Mat>> toSaveLeft;
queue<queue<Mat>> toSaveRight;

bool recording = false;
int vidCountLeft = 0;
int vidCountRight = 0;
string nameL = "VID_0";
string nameR = "VID_0";

VideoWriter outLeft("FootageL/VID_0.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 60, S);
VideoWriter outRight("FootageR/VID_0.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 60, S);

void processVideo(VideoCapture& cap, bool left) {
	Mat frame;
	cap.read(frame);

	if (left) leftCam.push(frame);
	else rightCam.push(frame);
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
              cpu_set_t set;
              CPU_ZERO(&set);
              CPU_SET(0, &set);
              
              pthread_t thread;
              thread = pthread_self();
              pthread_setaffinity_np(thread, sizeof(cpu_set_t), &set);
              processVideo(p_captureL, true);
             }
            break;
          case 1:
             if (recording) {
              cpu_set_t set;
              CPU_ZERO(&set);
              CPU_SET(1, &set);
              
              pthread_t thread;
              thread = pthread_self();
              pthread_setaffinity_np(thread, sizeof(cpu_set_t), &set);
              processVideo(p_captureR, false);
             }
            break;
          case 2:
            if (!toSaveLeft.empty()) {
              cpu_set_t set;
              CPU_ZERO(&set);
              CPU_SET(2, &set);
              
              pthread_t thread;
              thread = pthread_self();
              pthread_setaffinity_np(thread, sizeof(cpu_set_t), &set);
              queue<Mat>& currSave = toSaveLeft.front();
              if (currSave.empty()) {
                cout << "FootageL/" + nameL + ".avi Completed" << endl;
                queue<Mat> empt;
		            swap(toSaveLeft.front(), empt);
                toSaveLeft.pop();
                outLeft.release();
              
                vidCountLeft++;
                nameL = "VID_" + to_string(vidCountLeft);
                
                string vidName = "FootageL/" + nameL + ".avi";
                outLeft = VideoWriter(vidName, VideoWriter::fourcc('M', 'J', 'P', 'G'), 60, S);
              }
              else {
                outLeft.write(currSave.front());
                currSave.front().deallocate();
                currSave.pop();
              }
            }
            break;
          case 3:
            if (!toSaveRight.empty()) {
              cpu_set_t set;
              CPU_ZERO(&set);
              CPU_SET(3, &set);
              
              pthread_t thread;
              thread = pthread_self();
              pthread_setaffinity_np(thread, sizeof(cpu_set_t), &set);
              queue<Mat>& currSave = toSaveRight.front();
              if (currSave.empty()) {
                cout << "FootageR/" + nameR + ".avi Completed" << endl;
                queue<Mat> empt;
		            swap(toSaveRight.front(), empt);
		            
                toSaveRight.pop();
                outRight.release();
              
                vidCountRight++;
                nameR = "VID_" + to_string(vidCountRight);
                
                string vidName = "FootageR/" + nameR + ".avi";
                outRight = VideoWriter(vidName, VideoWriter::fourcc('M', 'J', 'P', 'G'), 60, S);
              }
              else {
                outRight.write(currSave.front());
                
                currSave.front().deallocate();
                cout << currSave.front().total() << endl;
                currSave.pop();
              }
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
	
	MainEvent mainEvent(captureL, captureR);

	cout << "Video Ready" << endl;
  Mat placeHolder = Mat::zeros(1, 1, CV_8S);

  double avgCount =  0;
  double avgTotal = 0;
	while (1) {
	
	  int64 start = getTickCount();
	   
		parallel_for_(Range(0, 4), mainEvent);
		
		if(recording){
		    double fps = getTickFrequency() / (getTickCount() - start);
		    avgCount++;
		    avgTotal += fps;
		    double newAvg = avgTotal / avgCount;
		    //cout << "Running FPS avg: " << newAvg << endl;
		    cout << "Current FPS: " << fps << endl;
		    
		}
		
		
		imshow("s", placeHolder);
		
		char letter = waitKey(1);
		if (letter == ' ') {
		  if (recording) {
		    cout << "Recoding Stopped" << endl;
		    toSaveLeft.push(leftCam);
		    toSaveRight.push(rightCam);
		    queue<Mat> emptL, emptR;
		    
		    swap(leftCam, emptL);
		    swap(rightCam, emptR);
		  }
		  else cout << "Recording Started" << endl;
		  
		  recording = !recording;
		}
	}
}

void showCamera2() {
	VideoCapture captureL("/dev/video" + to_string(1), CAP_V4L2);
	//VideoCapture captureR("v4l2src device=/dev/video2 ! video/x-raw, width=(int)1280, height=(int)720, format=(string)I420, framerate=(fraction)60/1 ! \
			videoconvert ! video/x-raw, format=(string)BGR ! \
			appsink");
}*/

void recordLeft() {
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(1, &set);
  
  pthread_t thread;
  thread = pthread_self();
  pthread_setaffinity_np(thread, sizeof(cpu_set_t), &set);
  
	VideoCapture captureL("/dev/video" + to_string(1), CAP_V4L2);
	VideoWriter outLeft("../out.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 60, S);

	captureL.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
	captureL.set(CAP_PROP_FPS, 60);
	captureL.set(CAP_PROP_FRAME_WIDTH, 1280);
	captureL.set(CAP_PROP_FRAME_HEIGHT, 720);
	
	vector<Mat> outs;
	
	while(1) {
    Mat left;
    captureL.read(left);
    
    if (outs.size() > 600) {
      for (Mat o : outs) outLeft << o;
      outs.clear();
      break;
    }
    else {
      outs.push_back(left.clone());
    }
  }

}

void recordRight() {
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(2, &set);
  
  pthread_t thread;
  thread = pthread_self();
  pthread_setaffinity_np(thread, sizeof(cpu_set_t), &set);
  
	VideoCapture captureL("/dev/video" + to_string(2), CAP_V4L2);
	VideoWriter outLeft("../out1.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 60, S);

	captureL.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
	captureL.set(CAP_PROP_FPS, 60);
	captureL.set(CAP_PROP_FRAME_WIDTH, 1280);
	captureL.set(CAP_PROP_FRAME_HEIGHT, 720);
	
	vector<Mat> outs;
	
	while(1) {
    Mat left;
    captureL.read(left);
    
    if (outs.size() > 600) {
      for (Mat o : outs) outLeft << o;
      outs.clear();
      break;
    }
    else {
      outs.push_back(left.clone());
    }
  }

}

int main() {
  thread cameraL(recordLeft);
  thread cameraR(recordRight);
  
  cameraL.join();
  cameraR.join();
 
  //collectFrame();
  
	return 0;
}
