#include <iostream>

#include "constants.hpp"

using namespace std;

namespace LGR {
  struct Axis {
    string name;
  
    double p1;
    double p2;
    double p3;
    
    double pf;
    double vel;
    
    void stepTime(double newP1);
    double calcVel(double dt);
    
    void print();
    void reset();
    
    Axis(string n) : name(n) {}
  };
  
  void Axis::stepTime(double newP1) {
    p3 = p2;
    p2 = p1;
    p1 = newP1;
  }
  
  double Axis::calcVel(double td) {
    vel = (p2 - p1)/td;
    return vel;
  }
  
  void Axis::reset() {
    p3 = p1;
    p2 = p1;
    pf = p1;
    
    vel = 0;
  }
  
  void Axis::print() {
    cout << name + "f: " << pf << ", " + name + " Vel: " << vel << endl;
  }
}
