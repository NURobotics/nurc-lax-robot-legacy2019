#include <sstream>
#include <string>
#include <iostream>
#include <ctime>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/videoio/videoio.hpp"

using namespace cv;
using namespace std;
using namespace std::chrono;


// Initial min and max HSV filter values.
int H_MIN = 0;
int H_MAX = 255;
int S_MIN = 0;
int S_MAX = 255;
int V_MIN = 0;
int V_MAX = 255;

// Default capture width and height
//const int FRAME_WIDTH = 1920; const int FRAME_HEIGHT = 1080;
const int FRAME_WIDTH = 1280; const int FRAME_HEIGHT = 720;
//const int FRAME_WIDTH = 640; const int FRAME_HEIGHT = 480;

// Max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;

// Minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

// Creates structuring element that will be used to "dilate" and "erode" image.
const Mat erodeElement = getStructuringElement(MORPH_RECT,Size(6,6));
const Mat dilateElement = getStructuringElement(MORPH_RECT,Size(16,16));

// Names that will appear at the top of each window
const string windowNameL = "Left Original Image";
const string windowNameL1 = "Left HSV Image";
const string windowNameL2 = "Left Thresholded Image";

const string windowNameR = "Right Original Image";
const string windowNameR1 = "Right HSV Image";
const string windowNameR2 = "Right Thresholded Image";
const string trackbarWindowName = "Trackbars";

// Measures the time of each function
double frameCounter, readt, cvtColort, inRanget, morphOpst, trackFilteredObjectt, findCirlcest = 0;

vector<vector<double>> cameraMatrix {{1097.851794859511983 , 0, 962.617161028923874},
                                     {0, 1102.540859860303271, 510.948016501172276},
                                     {0, 0, 1}};
vector<double> distCoeffs {-0.344362174612084 , 0.082330596538753 , 0.000331918687119 , -0.000348459718033 , 0.000000000000000};

void on_trackbar(int, void*) {}

string intToString(int number) {
	stringstream ss;
	ss << number;
	return ss.str();
}


double timeOfFunction(high_resolution_clock::time_point start, string func) {
	high_resolution_clock::time_point finish = high_resolution_clock::now();
	double elapsedMilliseconds = duration_cast<milliseconds>(finish - start).count() / 1000.0 ;
	return elapsedMilliseconds;
	//cout << func << ": " << elapsedMilliseconds << endl;
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
    erode(thresh,thresh,erodeElement);
	dilate(thresh,thresh,dilateElement);
}

/*void findCirlces(bool SCH, Mat &thresh, Mat &cameraFeed, vector<Vec3f> circles) {

    medianBlur(thresh, thresh, 5);

    HoughCircles(thresh, circles, HOUGH_GRADIENT, 1,
                 thresh.rows/16,
                 100, 13, 10, 0);

    // Draws circles around the balls
    for (int i = 0; i < circles.size() && SCH; i++) {
        Vec3i c = circles[i];
        Point center = Point(c[0], c[1]);
        // circle center
        circle( cameraFeed, center, 1, Scalar(255,0,255), 3, LINE_AA);
        // circle outline
        int radius = c[2];
        circle( cameraFeed, center, radius, Scalar(255,0,255), 3, LINE_AA);
    }
}*/

void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed, bool &objectFound, bool SCH) {

    //these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;

	// Finds contours of filtered image
	findContours(threshold,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);

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
			if (objectFound && SCH) {
				putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
				drawObject(x,y,cameraFeed);
            }
		}
	}
}

void GetCooridnates(VideoCapture capture, int* x, int* y, Mat* cameraFeed, Mat* HSV, Mat* threshold, bool* objectFound, bool showCrossHairs, bool timeIt) {
	vector<Vec3f> circles;

    // Stores the image to matrix
    high_resolution_clock::time_point start = high_resolution_clock::now();
    Mat temp;
    capture.read(*cameraFeed);
    if (timeIt) readt += timeOfFunction(start, "read");

    //Undistorts Camera Images
    //undistort(temp, *cameraFeed, cameraMatrix, distCoeffs);

    // Converts frame from BGR to HSV colorspace
    if (timeIt) start = high_resolution_clock::now();
    cvtColor(*cameraFeed,*HSV,COLOR_BGR2HSV);
    if (timeIt) cvtColort += timeOfFunction(start, "cvtColor");

    // Filters HSV image between values and store filtered image to
    // threshold matrix
    if (timeIt) start = high_resolution_clock::now();
    inRange(*HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),*threshold);
    if (timeIt) inRanget += timeOfFunction(start, "inRange");

    // Performs morphological operations on thresholded image to eliminate noise
    // and emphasize the filtered object(s)
    if (timeIt) start = high_resolution_clock::now();
    morphOps(*threshold);
    if (timeIt) morphOpst += timeOfFunction(start, "morphOps");

    // Passes in thresholded frame to our object tracking function
    // this function will return the x and y coordinates of the
    // filtered object
    if (timeIt) start = high_resolution_clock::now();
    trackFilteredObject(*x,*y,*threshold,*cameraFeed,*objectFound,showCrossHairs);
    if (timeIt) trackFilteredObjectt += timeOfFunction(start, "trackFilteredObject");

    if (timeIt) start = high_resolution_clock::now();
    //findCirlces(showCrossHairs, threshold, cameraFeed, circles);
    if (timeIt) findCirlcest += timeOfFunction(start, "findCircles");
}

int main(int argc, char* argv[]) {

    // 1=Left 2=Right
	int x1, y1 = 0;
	int x2, y2 = 0;

	Mat cameraFeed1, cameraFeed2;
	Mat HSV1, HSV2;
	Mat threshold1, threshold2;

	bool objectFound1 = false;
	bool objectFound2 = false;
	bool showCrossHairs = true;

	VideoCapture capture1(1);
	VideoCapture capture2(2);

    capture1.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	capture1.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture1.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

    capture2.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	capture2.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture2.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

    time_t timeBegin = time(0);
    int tick = 0;

	createTrackbars();
	namedWindow(windowNameL, WINDOW_NORMAL);
	namedWindow(windowNameL2, WINDOW_NORMAL);
	namedWindow(windowNameL1, WINDOW_NORMAL);

	while(1) {
        GetCooridnates(capture1, &x1, &y1, &cameraFeed1, &HSV1, &threshold1, &objectFound1, showCrossHairs, true);
	//capture2.read(cameraFeed2);
        //GetCooridnates(capture2, &x2, &y2, &cameraFeed2, &HSV2, &threshold2, &objectFound2, showCrossHairs, false);

        //camera1.join();
        //camera2.join();

        frameCounter++;

        time_t timeNow = time(0) - timeBegin;
        if (timeNow - tick >= 1) {
            tick++;
            cout << "Frames per second: " << frameCounter << endl;
            cout << "Read: " << readt / frameCounter << endl;
            cout << "cvtColor: " << cvtColort / frameCounter << endl;
            cout << "inRange: " << inRanget / frameCounter << endl;
            cout << "morphOpst: " << morphOpst / frameCounter << endl;
            cout << "tFO: " << trackFilteredObjectt / frameCounter << endl;
            cout << "findCirlces: " << findCirlcest / frameCounter << endl;
            frameCounter = 0; readt = 0; cvtColort = 0; inRanget = 0; morphOpst = 0; trackFilteredObjectt = 0, findCirlcest = 0;
        }

        if (objectFound1)
            cout << "X1: " << x1 << " / Y1: " << y1 << endl;
        if (objectFound2)
            cout << "X2: " << x2 << " / Y2: " << y2 << endl;

        // Show Frames if caps lock is on
            imshow(windowNameL,cameraFeed1); imshow(windowNameL2,threshold1); imshow(windowNameL1,HSV1); waitKey(1);
            //imshow(windowNameR,cameraFeed2); 
//imshow(windowNameR2,threshold2); imshow(windowNameR1,HSV2); waitKey(1);
	}

	return 0;
}
