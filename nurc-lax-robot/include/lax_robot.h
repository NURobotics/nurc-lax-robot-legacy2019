#include <string>
#include <RoboteqController.h>
#include <pixyhandle.hpp>

class LaxRobot {
private:
	//two cameras and one controller
	PixyHandle camera_arr_[2];
	RoboteqController controller_;

	//data strucutres to handle data



public:
	LaxRobot(PixyHandle c0, PixyHandle c1, RoboteqController controller); //init function
	~LaxRobot() {}; //teardown function

};


