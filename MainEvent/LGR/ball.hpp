#include "constants.hpp"
#include <iostream>
#include <math.h>

using namespace std;

namespace LGR {
struct Ball {
  Axis X = Axis("X");
  Axis Y = Axis("Y");
  Axis Z = Axis("Z");

  TickMeter t;

  int consecutiveFrames = 0;

  void storeNewCoordinate(double newX, double newY, double newZ);
  void calcFinalPos(double dt);

  void setRobotPosition(Robot &robot);
  void print();
  void reset();
};

//TODO: Finish/Correct This
void Ball::calcFinalPos(double dt) {
  if (consecutiveFrames < 2) {
    return;
  }
  if (consecutiveFrames == 2) {
    X.calcVelocity(dt);
    Y.calcVelocity(dt);
    Z.calcVelocity(dt);

    double finalDT = abs((Z.frame1) / Z.velocity);

    X.final = (X.velocity * finalDT) * (1 - EXP_AVG_WEIGHT) +
              X.final * EXP_AVG_WEIGHT;
    Y.final = (0.5 * GRAVITY * (finalDT * finalDT) + Y.velocity * finalDT +
               Y.frame0) *
                  (1 - EXP_AVG_WEIGHT) +
              Y.final * EXP_AVG_WEIGHT;
  } else if (consecutiveFrames > 2) {
    Z.calcVelocity(dt);
    //TODO: Remove this dumb equation and replace with simple kinematics eq.
    //Also this could just be its own freaking function
    X.final = ((Z.frame1 * Z.frame2 * (Z.frame1 - Z.frame2) * X.frame0 +
                Z.frame2 * Z.frame0 * (Z.frame2 - Z.frame0) * X.frame1 +
                Z.frame0 * Z.frame1 * (Z.frame0 - Z.frame1) * X.frame2) /
               ((Z.frame0 - Z.frame1) * (Z.frame0 - Z.frame2) *
                (Z.frame1 - Z.frame2))) *
                  (1 - EXP_AVG_WEIGHT) +
              X.final * EXP_AVG_WEIGHT;
    Y.final = ((Z.frame1 * Z.frame2 * (Z.frame1 - Z.frame2) * Y.frame0 +
                Z.frame2 * Z.frame0 * (Z.frame2 - Z.frame0) * Y.frame1 +
                Z.frame0 * Z.frame1 * (Z.frame0 - Z.frame1) * Y.frame2) /
               ((Z.frame0 - Z.frame1) * (Z.frame0 - Z.frame2) *
                (Z.frame1 - Z.frame2))) *
                  (1 - EXP_AVG_WEIGHT) +
              Y.final * EXP_AVG_WEIGHT;
  }

  if (Y.final < robot.CameraHeight) {
    Y.final = -robot.CameraHeight - (Y.final + robot.CameraHeight);
  }
}

void Ball::setRobotPosition(Robot &robot) {
  robot.DestinationX = X.final;
  robot.DestinationY = Y.final;

  if (Z.velocity < MIN_SPEED) {
    robot.DestinationX = X.frame0;
    robot.DestinationY = Y.frame0;
  }
}
void Ball::storeNewCoordinate(double newX, double newY, double newZ) {
  X.storeNewCoordinate(newX);
  Y.storeNewCoordinate(newY);
  Z.storeNewCoordinate(newZ);

  consecutiveFrames++;
}

void Ball::reset() {
  X.reset();
  Y.reset();
  Z.reset();

  consecutiveFrames = 0;
}

void Ball::print() {
  X.print();
  Y.print();
  Z.print();
}
} // namespace LGR
