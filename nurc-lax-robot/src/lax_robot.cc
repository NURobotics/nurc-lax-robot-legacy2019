#include <string>
#include <lax_robot.h>

LaxRobot::LaxRobot(PixyHandle c0, PixyHandle c1, RoboteqController controller) {
	camera_arr_[0] = c0;
	camera_arr_[1] = c1;
	controller_ = controller;

}