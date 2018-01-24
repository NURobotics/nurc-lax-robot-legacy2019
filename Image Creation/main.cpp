#include <sstream>
#include <string>
#include <iostream>
#include <ctime>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;


// Initial min and max HSV filter values.
int H_MIN = 0;
int H_MAX = 255;
int S_MIN = 0;
int S_MAX = 255;
int V_MIN = 0;
int V_MAX = 255;

// Default capture width and height
const int FRAME_WIDTH = 1920; const int FRAME_HEIGHT = 1080;
//const int FRAME_WIDTH = 1280; const int FRAME_HEIGHT = 720;
//const int FRAME_WIDTH = 320; const int FRAME_HEIGHT = 240;

// Max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;

// Minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

// Names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string trackbarWindowName = "Trackbars";

void on_trackbar(int, void*) {}

string intToString(int number) {
	stringstream ss;
	ss << number;
	return ss.str();
}

void createTrackbars() {

    namedWindow(trackbarWindowName,0);

	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN", H_MIN);
	sprintf( TrackbarName, "H_MAX", H_MAX);
	sprintf( TrackbarName, "S_MIN", S_MIN);
	sprintf( TrackbarName, "S_MAX", S_MAX);
	sprintf( TrackbarName, "V_MIN", V_MIN);
	sprintf( TrackbarName, "V_MAX", V_MAX);

    createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
    createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
    createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
    createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
    createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
    createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
}

void drawObject(int x, int y,Mat &frame) {
	circle(frame,cvPoint(x,y),20,cvScalar(0,255,0),2);
    if(y-25>0)
    line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
    if(y+25<FRAME_HEIGHT)
    line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
    if(x-25>0)
    line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
    if(x+25<FRAME_WIDTH)
    line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);

	putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);
}

void morphOps(Mat &thresh) {
	// Creates structuring element that will be used to "dilate" and "erode" image.
	Mat erodeElement = getStructuringElement(MORPH_RECT,Size(6,6));
	Mat dilateElement = getStructuringElement(MORPH_RECT,Size(16,16));

	erode(thresh,thresh,erodeElement);
	dilate(thresh,thresh,dilateElement);
}

void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed, bool &objectFound){

	Mat temp;
	threshold.copyTo(temp);

	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;

	// Finds contours of filtered image
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );

	// Uses moments method to find our filtered object
	double refArea = 0;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();

        // Image too noisy if number of objects >= MAX_NUM_OBJECTS
        if (numObjects < MAX_NUM_OBJECTS){
			for (int i = 0; i >= 0; i = hierarchy[i][0]) {

				Moments moment = moments((cv::Mat)contours[i]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we save a reference area of each
				//iteration and compare it to the area in the next iteration.
                if (area > MIN_OBJECT_AREA && area < MAX_OBJECT_AREA && area > refArea) {
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;
					refArea = area;
				} else objectFound = false;
			}

			// Draws the cross-hairs on the window
			if (objectFound) {
				putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
				drawObject(x,y,cameraFeed);
            }
		}
	}
}

int main(int argc, char* argv[]) {

	Mat cameraFeed;
	Mat HSV;
	Mat threshold;

	int x=0, y=0;
	bool objectFound = false;

	double frameCounter = 0;
    time_t timeBegin = time(0);
    int tick = 0;

	createTrackbars();
	VideoCapture capture(1);

	capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

	while(capture.isOpened()) {
		// Stores the image to matrix
		capture.retrieve(cameraFeed);

		// Converts frame from BGR to HSV colorspace
		cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);

		// Filters HSV image between values and store filtered image to
		// threshold matrix
		inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);

		// Performs morphological operations on thresholded image to eliminate noise
		// and emphasize the filtered object(s)
        morphOps(threshold);

		// Passes in thresholded frame to our object tracking function
		// this function will return the x and y coordinates of the
		// filtered object
        trackFilteredObject(x,y,threshold,cameraFeed,objectFound);

        frameCounter++;

        time_t timeNow = time(0) - timeBegin;
        if (timeNow - tick >= 1) {
            tick++;
            cout << "Frames per second: " << frameCounter << endl;
            frameCounter = 0;
        }

        if (objectFound)
            cout << "X: " << x << " / Y: " << y << endl;

		//show frames
		imshow(windowName2,threshold);
		imshow(windowName,cameraFeed);
		imshow(windowName1,HSV);

		// Delay 3ms so that screen can refresh.
		// Image will not appear without this waitKey() command
		// Delete when don't need to display windows
		waitKey(3);
	}
	return 0;
}
