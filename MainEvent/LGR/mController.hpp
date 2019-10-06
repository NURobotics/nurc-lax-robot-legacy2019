#include <iostream>
#include <opencv2/core/core.hpp>

#include "../Roboteq/RoboteqDevice.cpp"
#include "../Roboteq/ErrorCodes.h"
#include "../Roboteq/Constants.h"
#include "constants.hpp"

using namespace std;

namespace LGR {
  struct mController {
    const int innerIndex = 1;
    const int outerIndex = 2;

    const int innerRPM = 747;
    const int outerRPM = 747;

    const int innerStepCount = 2000;
    const int outerStepCount = 2000;

    const string port = "\\\\.\\com3";

    RoboteqDevice RD;

    int innerCount;
    int outerCount;

    void calcCounts(Robot& r);
    void sendAngles();
    void configure();

    void print();
    void reset();
  };

  void mController::calcCounts(Robot& r) {
    double hyp = sqrt(r.DestX*r.DestX + r.DestY*r.DestX);
    if (hyp > r.maxArmLen) hyp = r.maxArmLen;
    if (hyp < r.minArmLen) hyp = r.minArmLen;

    r.Motor2Angle = acos((hyp*hyp - r.squareSumArmLen)/(r.outInArm2));
    r.Motor1Angle = atan2(r.DestY, r.DestX) - atan((r.outerArmLen*sin(r.Motor2Angle))/(r.innerArmLen+r.outerArmLen*cos(r.Motor2Angle)));
    r.Motor2Angle += r.Motor1Angle;

    if (r.DestX < 0) {
        r.Motor1Angle += CV_PI;
        r.Motor2Angle += CV_PI;
    }

    innerCount = ((r.DestX > 0 ? CV_PI : 0) + atan(r.DestY/r.DestX))/(2*CV_PI) * innerStepCount;
    outerCount = r.Motor2Angle/(2*CV_PI) * outerStepCount;
  }

  void mController::sendAngles() {
    RD.SetCommand(_P, innerIndex, innerCount);
    RD.SetCommand(_P, outerIndex, outerCount);
  }

  void mController::configure() {
    int status;
    cout << "Connecting to Roboteq...";
    if((status = RD.Connect(port)) != RQ_SUCCESS){
      cout << "Error connecting to device --> " << status << "." << endl;
      return;
    }
    else {
      cout << "succeeded!";
    }
  }

  void mController::reset() {
    innerCount = 0;
    outerCount = 0;
  }

  void mController::print() {
    cout << "Inner Count:  " << innerCount << ", Outer Count: " << innerCount << endl;
  }
}
