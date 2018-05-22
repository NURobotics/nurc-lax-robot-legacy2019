#include <sstream>
#include <string>
#include <iostream>
#include <thread>
#include <Windows.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

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
//const int FRAME_WIDTH = 1920; const int FRAME_HEIGHT = 1080;
const int FRAME_WIDTH = 1280; const int FRAME_HEIGHT = 720;
//const int FRAME_WIDTH = 640; const int FRAME_HEIGHT = 480;

// Max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 75;

// Minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

// Creates structuring element that will be used to "dilate" and "erode" image.
const Mat erodeElement = getStructuringElement(MORPH_RECT,Size(6,6));
const Mat dilateElement = getStructuringElement(MORPH_RECT,Size(16,16));

// Declaring Time Variables
double ti = 0;
double t2 = 0;
double td = 0;
double tf = 0;

//Declaring Velocity and Gravity
double g = 9.80665;
double zv = 0;
double xv = 0;
double yv = 0;

// Declaring Z Variables
double zi = 0;
double z2 = 0;
double z3 = 0;
double zf = 0;

// Declaring X Variables
double xi = 0;
double x2 = 0;
double x3 = 0;
double xf = 0;

// Declaring Y Variables
double yi = 0;
double y2 = 0;
double y3 = 0;
double yf = 0;

// Declaring distance from center joint to position (hypotenuse) and angles
double hyp = 0;
double angle1 = 0;
double angle2 = 0;

// Declaring image holders
Mat cameraFeedL, cameraFeedR;
Mat HSVL, HSVR;
Mat thresholdL, thresholdR;

Mat ballCoordL(1,1,CV_64FC2);
Mat ballCoordR(1,1,CV_64FC2);
Mat undistoredCoordL;
Mat undistoredCoordR;

Mat P(4,1,CV_64F);

// Distances in physical space (m)
double dxCameras = 0.362;
double dyGround = 1;
double innerArmLength = .5;
double outerArmLength = .5;
double maxArmLength;
double minArmLength;

Mat KL(3,3,CV_64F, Scalar(0.0));
Mat KR(3,3,CV_64F, Scalar(0.0));

Mat distortion_coeffsL(5,1,CV_64F);
Mat distortion_coeffsR(5,1,CV_64F);

Mat transformL(3,4,CV_64F, Scalar(0.0));
Mat transformR(3,4,CV_64F, Scalar(0.0));

int amountFound = 0;
bool objectFoundL = false;
bool objectFoundR = false;
bool showCrossHairs = true;
bool showWindows = false;

// Names that will appear at the top of each window
const string windowNameL = "Left Original Image";
const string windowNameL1 = "Left HSV Image";
const string windowNameL2 = "Left Thresholded Image";

const string windowNameR = "Right Original Image";
const string windowNameR1 = "Right HSV Image";
const string windowNameR2 = "Right Thresholded Image";
const string trackbarWindowName = "Trackbars";

// Measures the time of each function
double frameCounter, readt, cvtColort, inRanget, morphOpst, trackFilteredObjectt, findCirlcest, calcPositiont = 0;


void on_trackbar(int, void*) {}

string intToString(int number) {
	stringstream ss;
	ss << number;
	return ss.str();
}

inline long long PerformanceCounter()
{
    LARGE_INTEGER li;
    ::QueryPerformanceCounter(&li);
    return li.QuadPart;
}

inline long long PerformanceFrequency()
{
    LARGE_INTEGER li;
    ::QueryPerformanceFrequency(&li);
    return li.QuadPart;
}

double timeOfFunction(long long start, string func) {
	long long finish = PerformanceCounter();
	double elapsedMilliseconds = ((finish - start) * 1000.0) / PerformanceFrequency();
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

void findCirlces(bool SCH, Mat &thresh, Mat &cameraFeed, vector<Vec3f> circles) {

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
}

void trackFilteredObject(double &x, double &y, Mat threshold, Mat &cameraFeed, bool &objectFound, bool SCH) {

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

/*
* Need to factor in ball bounce
*   Set so if ball's yf is below or at the ground (defined by the distance between the cameras and ground), have it make yf = groundY - (yf - groundY)
* Need to do parabolic trajectory to account for curve shots
*   If we have three points, use the following to find C in u=Av^2+Bv+C
        D = (v1 - v2)(v1 - v3)(v2 - v3)
        C = (v2 * v3 * (v2 - v3) * u1 + v3 * v1 * (v3 - v1) * u2 + v1 * v2 * (v1 - v2) * u3) / D
*   Do once with u=x,v=z and another with u=y,v=z and set the value of C to xf and yf since the ball reaches the robot at z=0
*/
void findFinalAndAngles() {
    if (amountFound == 2) {
        td = t2 - ti;

        zv = (z2 - zi)/td;
        xv = (x2 - xi)/td;
        yv = (y2 - yi)/td;

        tf = (zf - zi)/zv + ti;

        xf = xv * tf;
        yf = 0.5 * g * (tf*tf) + yv * tf + yi;
    }
    else if (amountFound > 2) {
        xf = (z2 * z3 * (z2 - z3) * xi + z3 * zi * (z3 - zi) * x2 + zi * z2 * (zi - z2) * x3) / ((zi - z2) * (zi - z3) * (z2 - z3));
        yf = (z2 * z3 * (z2 - z3) * yi + z3 * zi * (z3 - zi) * y2 + zi * z2 * (zi - z2) * y3) / ((zi - z2) * (zi - z3) * (z2 - z3));
    }

    if (yf < -dyGround) {
        yf = -dyGround - (yf + dyGround);
    }

    hyp = sqrt(xf*xf + yf*yf);
    if (hyp > maxArmLength) hyp = maxArmLength;
    if (hyp < minArmLength) hyp = minArmLength;

    angle2 = acos((hyp*hyp - innerArmLength*innerArmLength - outerArmLength*outerArmLength)/(2*innerArmLength*outerArmLength));
    angle1 = atan2(yf,xf) - atan((outerArmLength*sin(angle2))/(innerArmLength+outerArmLength*cos(angle2)));
}

void GetCooridnates(VideoCapture capture, double* x, double* y, Mat* cameraFeed, Mat* HSV, Mat* threshold, bool* objectFound, bool showCrossHairs, bool LeftCam) {
	vector<Vec3f> circles;

    // Stores the image to matrix
    long long start = PerformanceCounter();
    capture.read(*cameraFeed);
    if (LeftCam) readt += timeOfFunction(start, "read");
    //if (LeftCam) *cameraFeed = *cameraFeed*1;

    // Converts frame from BGR to HSV colorspace
    if (LeftCam) start = PerformanceCounter();
    cvtColor(*cameraFeed,*HSV,COLOR_BGR2HSV);
    if (LeftCam) cvtColort += timeOfFunction(start, "cvtColor");

    // Filters HSV image between values and store filtered image to
    // threshold matrix
    if (LeftCam) start = PerformanceCounter();
    inRange(*HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),*threshold);
    if (LeftCam) inRanget += timeOfFunction(start, "inRange");

    // Performs morphological operations on thresholded image to eliminate noise
    // and emphasize the filtered object(s)
    if (LeftCam) start = PerformanceCounter();
    morphOps(*threshold);
    if (LeftCam) morphOpst += timeOfFunction(start, "morphOps");

    // Passes in thresholded frame to our object tracking function
    // this function will return the x and y coordinates of the
    // filtered object
    if (LeftCam) start = PerformanceCounter();
    trackFilteredObject(*x,*y,*threshold,*cameraFeed,*objectFound,showCrossHairs);
    if (LeftCam) trackFilteredObjectt += timeOfFunction(start, "trackFilteredObject");

    if (LeftCam) start = PerformanceCounter();
    //findCirlces(showCrossHairs, threshold, cameraFeed, circles);
    if (LeftCam) findCirlcest += timeOfFunction(start, "findCircles");
}

void findRealandPosition(double xcL, double ycL, double xcR, double ycR, Mat projL, Mat projR) {
    long long start = PerformanceCounter();
    if (objectFoundL && objectFoundR) {
        amountFound++;

        ballCoordL.at<Vec2d>(0,0)[0] = xcL;
        ballCoordL.at<Vec2d>(0,0)[1] = ycL;
        ballCoordR.at<Vec2d>(0,0)[0] = xcR;
        ballCoordR.at<Vec2d>(0,0)[1] = ycR;

        undistortPoints(ballCoordL,undistoredCoordL,KL,distortion_coeffsL);
        undistortPoints(ballCoordR,undistoredCoordR,KR,distortion_coeffsR);

        triangulatePoints(projL, projR, ballCoordL, ballCoordR, P);

        x3 = x2; y3 = y2; z3 = z2;
        x2 = xi; y2 = yi; z2 = zi; t2 = ti;

        xi = -P.at<double>(0,0)/P.at<double>(3,0) + dxCameras/2;//triangulate points outputs;
        yi = -P.at<double>(1,0)/P.at<double>(3,0);//triangulate points outputs;
        zi = -P.at<double>(2,0)/P.at<double>(3,0);//triangulate points outputs;
        ti = (PerformanceCounter() / PerformanceFrequency()) * 1000000.0;

        findFinalAndAngles();
    }
    else {
        amountFound = 0;
    }
    calcPositiont += timeOfFunction(start, "calcPosition");
}

int main(int argc, char* argv[]) {

    // Old camera = RIGHT
	double xcL, ycL = 0;
	double xcR, ycR = 0;

    KL.at<double>(0,0) = 1072.596243797988791;
    KL.at<double>(1,1) = 1073.690381426655222;
    KL.at<double>(0,2) = 680.723893064209733;
    KL.at<double>(1,2) = 383.222991477372091;
    KL.at<double>(2,2) = 1;

    KR.at<double>(0,0) = 1051.570474935752145;
    KR.at<double>(1,1) = 1052.831434088737069;
    KR.at<double>(0,2) = 630.837501734280636;
    KR.at<double>(1,2) = 341.993415934465418;
    KR.at<double>(2,2) = 1;

    distortion_coeffsL.at<double>(0,0) = -0.407242377983198;
    distortion_coeffsL.at<double>(1,0) = 0.163506544703868;
    distortion_coeffsL.at<double>(2,0) = -0.000309830974174;
    distortion_coeffsL.at<double>(3,0) = 0.001234732556889;
    distortion_coeffsL.at<double>(4,0) = 0.000000000000000;

    distortion_coeffsR.at<double>(0,0) = -0.395033423474349;
    distortion_coeffsR.at<double>(1,0) = 0.157550161003169;
    distortion_coeffsR.at<double>(2,0) = 0.000064885496368;
    distortion_coeffsR.at<double>(3,0) = 0.000488116203253;
    distortion_coeffsR.at<double>(4,0) = 0.000000000000000;

    transformL.at<double>(0,0) = 1.0; transformR.at<double>(0,0) = 1.0;
    transformL.at<double>(1,1) = 1.0; transformR.at<double>(1,1) = 1.0;
    transformL.at<double>(2,2) = 1.0; transformR.at<double>(2,2) = 1.0;
    transformL.at<double>(0,3) = -dxCameras/2; transformR.at<double>(0,3) = dxCameras/2;

    Mat projL = KL*transformL;
    Mat projR = KR*transformR;

    maxArmLength = innerArmLength + outerArmLength;
    minArmLength = innerArmLength - outerArmLength;

	VideoCapture captureL(2);
	VideoCapture captureR(1);

    captureL.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	captureL.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	captureL.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

    captureR.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	captureR.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	captureR.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

    time_t timeBegin = time(0);
    int tick = 0;

	createTrackbars();
    namedWindow(windowNameL, WINDOW_NORMAL);
    namedWindow(windowNameL1, WINDOW_NORMAL);
    namedWindow(windowNameL2, WINDOW_NORMAL);
    namedWindow(windowNameR, WINDOW_NORMAL);
    namedWindow(windowNameR1, WINDOW_NORMAL);
    namedWindow(windowNameR2, WINDOW_NORMAL);

	while(1) {
        thread cameraL(GetCooridnates, captureL, &xcL, &ycL, &cameraFeedL, &HSVL, &thresholdL, &objectFoundL, showCrossHairs, true);
        thread cameraR(GetCooridnates, captureR, &xcR, &ycR, &cameraFeedR, &HSVR, &thresholdR, &objectFoundR, showCrossHairs, false);
        thread angles(findRealandPosition, xcL, ycL, xcR, ycR, projL, projR);

        cameraL.join();
        cameraR.join();
        angles.join();

        // Extra stuff used for testing time and showing windows

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
            cout << "calcPosition: " << calcPositiont / frameCounter << endl;
            cout << P/P.at<double>(3,0) << endl;
            cout << "xf: " << xf << ", yf: " << yf << endl;
            frameCounter = 0; readt = 0; cvtColort = 0; inRanget = 0; morphOpst = 0; trackFilteredObjectt = 0, findCirlcest = 0, calcPositiont = 0;
        }

        if (GetAsyncKeyState(VK_CAPITAL))
            showWindows = !showWindows;

        // Show Frames if caps lock is on
        if (showWindows) {
            imshow(windowNameL,cameraFeedL); imshow(windowNameL2,thresholdL); //imshow(windowNameL1,HSVL);
            imshow(windowNameR,cameraFeedR); imshow(windowNameR2,thresholdR); //imshow(windowNameR1,HSVR);

            resizeWindow(windowNameL, 640,360); resizeWindow(windowNameL2, 640,360); //resizeWindow(windowNameL1, 640,360);
            resizeWindow(windowNameR, 640,360); resizeWindow(windowNameR2, 640,360); //resizeWindow(windowNameR1, 640,360);

            moveWindow(windowNameL, 0,0); moveWindow(windowNameL2, 640,0); // moveWindow(windowNameL1, 720,0);
            moveWindow(windowNameR, 0,360); moveWindow(windowNameR2, 640,360); // moveWindow(windowNameR1, 720,360);

            waitKey(1);
        }
	}

	return 0;
}
