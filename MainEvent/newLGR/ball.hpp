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
    //Calculate Final Position
  }
}
