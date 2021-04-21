#include "constants.hpp"
#include <position.hpp>
#include <iostream>
#include <vector>
#include <math.h>

using namespace std;

namespace LGR {
  struct Ball {

    // KUBA TODO
    //Tracker one and two;

    vector<Position> position_history;
    Position current_position;
    Position projected_position;

    // Takes in right and left tracker
    // trackers can update themselves
    // takes in cv2::Mat KUBA TODO
    void update();
    void print();
    void reset();

  private:
    void map_to_3D();
    void calculate_final_position();
  };

  //TODO: Finish/Correct This
  void Ball::calcFinalPos(double dt) {
    //Calculate Final Position
  }
}
