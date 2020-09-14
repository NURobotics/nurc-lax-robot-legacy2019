#include <iostream>
#include <opencv2/core/core.hpp>

#include "constants.hpp"

using namespace std;

namespace LGR {
struct Robot {
  double Motor1Angle;
  double Motor2Angle;

  double DestinationX;
  double DestinationY;

  const double CameraSeperation = 0.362;
  const double CameraHeight = 0.75;

  const double innerArmLen = 0.05;
  const double outerArmLen = 0.05;

  const double squareSumArmLen =
      innerArmLen * innerArmLen + outerArmLen * outerArmLen;
  const double outInArm2 = 2 * innerArmLen * outerArmLen;
  const double maxArmLen = innerArmLen + outerArmLen;
  const double minArmLen = cv_abs(innerArmLen - outerArmLen);

  void draw();

  void print();
  void reset();
};

void Robot::reset() {
  Motor1Angle = 0;
  Motor2Angle = 0;
  DestinationX = 0;
  DestinationY = 0;
}

void Robot::print() {
  cout << "Motor Angles: 1 - " << Motor1Angle << ", 2 - " << Motor2Angle << endl
       << "X Destination: " << DestinationX
       << ", Y Destination: " << DestinationY << endl;
}

void Robot::draw() {}
} // namespace LGR
