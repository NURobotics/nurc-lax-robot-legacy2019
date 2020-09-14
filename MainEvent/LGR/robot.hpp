#include <iostream>
#include <opencv2/core/core.hpp>

#include "constants.hpp"

using namespace std;

namespace LGR {
struct Robot {
  double Motor1Angle;
  double Motor2Angle;

  double DestX;
  double DestY;

  const double Camera_dx_Camera = 0.362;
  const double Camera_dy_Ground = .75;

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
  DestX = 0;
  DestY = 0;
}

void Robot::print() {
  cout << "Motor Angles: 1 - " << Motor1Angle << ", 2 - " << Motor2Angle << endl
       << "X Destination: " << DestX << ", Y Destination: " << DestY << endl;
}

void Robot::draw() {}
} // namespace LGR
