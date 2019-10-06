using namespace std;

#define FRAME_WIDTH 1280
#define FRAME_HEIGHT 720
#define FRAMES_PER_SECOND 60

#define MAX_NUM_OBJECTS 45
#define MIN_OBJECT_AREA 100
#define MAX_OBJECT_AREA 614400 // FRAME_HEIGHT*FRAME_WIDTH/1.5

#define GRAVITY 9.80665

#define CONF_THRESHOLD 0.4 // Confidence threshold

// For OpenCV DNN
#define NMS_THRESHOLD 0.4 // Non-maximum suppression threshold
#define NET_SIZE 608 // Width and height of input image to network


//for ball location
#define EXP_AVG_WT 0.25
#define SLOW_TOL 0.5
