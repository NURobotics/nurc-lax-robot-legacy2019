
#include <iostream>
#include "constants.hpp"

namespace LGR {

  struct Position {
    double x, y, z;
    double time;
    Position(double x, double y, double z, double time)
      : x{x}, y{y}, z{z}, time{time}
    {}
  }

} // namespace LGR
