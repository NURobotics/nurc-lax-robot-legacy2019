#include <iostream>

#include "constants.hpp"

using namespace std;

namespace LGR {
struct Axis {
  string name;

  // 2 is most recent
  double frame0;
  double frame1;
  double frame2;

  double final;
  double velocity;

  void storeNewCoordinate(double newCoord);
  double calcVelocity(double dt);

  void print();
  void reset();

  Axis(string n) : name(n) {}
};

void Axis::storeNewCoordinate(double newCoord) {
  frame2 = frame1;
  frame1 = frame0;
  frame0 = newCoord;
}

double Axis::calcVelocity(double dt) {
  velocity = (frame1 - frame0) / dt;
  return velocity;
}

void Axis::reset() {
  frame2 = frame0;
  frame1 = frame0;
  final = frame0;

  velocity = 0;
}

void Axis::print() {
  cout << name + "f: " << final << ", " + name + " Vel: " << velocity << endl;
}
} // namespace LGR
