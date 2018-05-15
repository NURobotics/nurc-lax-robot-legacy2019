#include <string>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <roboteq_controller.h>
#include <pixyhandle.hpp>
#include <triangulation.h>

template <int N>
struct Coordinate {
  Coordinate() {}
  Coordinate(const Coordinate &coordinate) {
    memcpy(dim, coordinate.dim, sizeof(double) * N);
  }

  Coordinate& operator=(const Coordinate& other) {
    memcpy(dim, other.dim, sizeof(double) * N);
    return *this;
  }

  double& operator()(int index) {
    return dim[index];
  }

  double dim[N];
};

class LaxRobot {
public:
  static const int kHorizontalMaxPixel;
  static const int kVerticalMaxPixel;
  static const double kHorizontalFov;
  static const double kVerticalFov;
  static const int kMinBallArea;
  static const int kEncoderChannel;
  static const int kMotorChannel;

  LaxRobot(const std::string motor_port,
           const int motor_baud_rate);
	~LaxRobot();

  bool start();
  bool stop();

  bool ok();
  bool done();
  bool running();
  bool active();

private:
  void camera_worker();
  void motor_controller_worker();

  bool configure_motor_controller();
  void teardown_motor_controller();

  bool configure_cameras();
  void teardown_cameras();

  PixyHandle& left_camera() { return cameras_[left_camera_index_]; }
  PixyHandle& right_camera() { return cameras_[left_camera_index_]; }
  
	PixyHandle cameras_[2];
  Coordinate<3> camera_positions_[2];
	RoboteqController motor_controller_;

  int left_camera_index_;
  int right_camera_index_;

  boost::mutex flag_mutex_;
  bool ok_;
  bool done_;
  bool running_;

  boost::mutex position_mutex_;
  Coordinate<3> ball_position_;
  Coordinate<2> arm_target_position_;
  bool update_flag_;
  
  boost::thread camera_thread_;
  boost::thread motor_controller_thread_;

  boost::mutex debug_mutex_;
  void debug(const std::string debug_str);

  /**************************************/
  Coordinate<2> target_;
  float old_theta_;
  int spin_counter_;

};
