#include <iostream>
#include <opencv2/core/core.hpp>

#include "../Roboteq/Constants.h"
#include "../Roboteq/ErrorCodes.h"
#include "../Roboteq/RoboteqDevice.cpp"
#include "constants.hpp"

using namespace std;

namespace LGR {
struct mController {
  const int innerMotorIndex = 1;
  const int outerMotorIndex = 2;

  const int innerMotorRPM = 747;
  const int outerMotorRPM = 747;

  const int innerStepCount = 2000;
  const int outerStepCount = 2000;

  const string port = "\\\\.\\com3";

  RoboteqDevice roboteq;

  int innerMotorCount;
  int outerMotorCount;

  void calcCounts(Robot &robot);
  void sendAngles();
  void configure();

  void print();
  void reset();
};

void mController::calcCounts(Robot &robot) {
  double hypotenuse = sqrt(robot.DestinationX * robot.DestinationX +
                           robot.DestinationY * robot.DestinationX);
  if (hypotenuse > robot.maxArmLen)
    hypotenuse = robot.maxArmLen;
  if (hypotenuse < robot.minArmLen)
    hypotenuse = robot.minArmLen;

  robot.Motor2Angle = acos((hypotenuse * hypotenuse - robot.squareSumArmLen) /
                           (robot.outInArm2));
  robot.Motor1Angle =
      atan2(robot.DestinationY, robot.DestinationX) -
      atan((robot.outerArmLen * sin(robot.Motor2Angle)) /
           (robot.innerArmLen + robot.outerArmLen * cos(robot.Motor2Angle)));
  robot.Motor2Angle += robot.Motor1Angle;

  if (robot.DestinationX < 0) {
    robot.Motor1Angle += CV_PI;
    robot.Motor2Angle += CV_PI;
  }

  innerMotorCount = ((robot.DestinationX > 0 ? CV_PI : 0) +
                     atan(robot.DestinationY / robot.DestinationX)) /
                    (2 * CV_PI) * innerStepCount;
  outerMotorCount = robot.Motor2Angle / (2 * CV_PI) * outerStepCount;
}

void mController::sendAngles() {
  roboteq.SetCommand(_P, innerMotorIndex, innerMotorCount);
  roboteq.SetCommand(_P, outerMotorIndex, outerMotorCount);
}

void mController::configure() {
  int status;
  cout << "Connecting to Roboteq...";
  if ((status = roboteq.Connect(port)) != RQ_SUCCESS) {
    cout << "Error connecting to device --> " << status << "." << endl;
    return;
  } else {
    cout << "succeeded!";
  }
}

void mController::reset() {
  innerMotorCount = 0;
  outerMotorCount = 0;
}

void mController::print() {
  cout << "Inner Count:  " << innerMotorCount
       << ", Outer Count: " << innerMotorCount << endl;
}
} // namespace LGR
