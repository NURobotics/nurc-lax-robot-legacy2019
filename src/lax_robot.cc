#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>

#include <vector>

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <pixyhandle.hpp>
#include <lax_robot.h>

using boost::bind;
using boost::mutex;
using boost::thread;
using boost::unique_lock;
using std::cerr;
using std::endl;
using std::string;

LaxRobot::LaxRobot(
  const string motor_controller_port,
  const int motor_controller_baud_rate) :
  motor_controller_(motor_controller_port, motor_controller_baud_rate)
{
  debug("Constructing the Lax Robot");
  left_camera_index_ = 0;
  right_camera_index_ = 1;
  update_flag_ = false;
  done_ = false;
  ok_ = true;
  running_ = false;
}

LaxRobot::~LaxRobot()
{
  debug("Destructing the Lax Robot");
  stop();
}

void LaxRobot::debug(const std::string debug_str)
{
  unique_lock<mutex> debug_lock(debug_mutex_);
  cerr << "[Lax Robot] " << debug_str << endl;
}

void LaxRobot::camera_worker()
{
  debug("Starting up the camera worker");
  flag_mutex_.lock();
  while (ok_ && !done_) {
    flag_mutex_.unlock();
    
    debug("Running camera loop"); 
    usleep(100000);

    flag_mutex_.lock();
  }
  flag_mutex_.unlock();
}

void LaxRobot::motor_controller_worker()
{
  debug("Starting up the motor controller worker");
  flag_mutex_.lock();
  static Coordinate<2> target;
  bool updated = false;
  int encoder_count = 0;
  while (ok_ && !done_) {
    flag_mutex_.unlock();
    
    debug("Running motor controller loop"); 
    position_mutex_.lock();  
    updated = update_flag_;
    if (update_flag_) {
      target = arm_target_position_;
      update_flag_ = false;
    }
    position_mutex_.unlock();

    if (updated) {
      encoder_count = 2500; 
      motor_controller_.encoder_count_command(1, encoder_count);
    }

    flag_mutex_.lock();
  }
  flag_mutex_.unlock();
}

bool LaxRobot::configure_motor_controller()
{
  motor_controller_.open_serial();
  if (!motor_controller_.ok()) {
    debug("Could not open motor controller serial connection");
    return false;
  }

  if (!motor_controller_.set_echo(false) ||
      !motor_controller_.set_encoder_high_count_limit(1, 10000) ||
      !motor_controller_.set_encoder_low_count_limit(1, -10000) ||
      !motor_controller_.set_encoder_usage(1, 1, RoboteqController::FEEDBACK) ||
      !motor_controller_.set_encoder_ppr(1, 1250) ||
      !motor_controller_.set_amp_limit(1, 180) ||
      !motor_controller_.set_amp_trigger_level(1, 150) ||
      !motor_controller_.set_amp_trigger_delay(1, 1000) ||
      !motor_controller_.set_amp_trigger_action(1, RoboteqController::SAFETY_STOP) ||
      !motor_controller_.set_proportional_gain(1, 20) ||
      !motor_controller_.set_integral_gain(1, 0) ||
      !motor_controller_.set_differential_gain(1, 0) ||
      !motor_controller_.set_motor_acceleration(1, 170) ||
      !motor_controller_.set_motor_decceleration(1, 170) ||
      !motor_controller_.set_default_position_velocity(1, 100) ||
      !motor_controller_.set_max_rpm(1, 100) ||
      !motor_controller_.motor_position_mode_velocity_command(1, 100) ||
      !motor_controller_.set_operating_mode(1, RoboteqController::OPEN_LOOP_SPEED) ||
      !motor_controller_.encoder_count_command(1, 0) ||
      !motor_controller_.set_operating_mode(1, RoboteqController::CLOSED_LOOP_COUNT_POSITION)) {
    debug("Could not configure the motor controller with the default settings");
    return false;
  }
  return true;
}

bool LaxRobot::configure_cameras()
{
  return true;
}

bool LaxRobot::start()
{
  debug("Starting up the Lax Robot");

  if (!configure_motor_controller()) {
    debug("Failed to configure the motor controller");
    return false;
  }

  if (!configure_cameras()) {
    debug("Failed to configure the cameras");
    return false;
  }

  flag_mutex_.lock();
  running_ = true;
  flag_mutex_.unlock();

  camera_thread_ = thread(bind(&LaxRobot::camera_worker, this));
  motor_controller_thread_ = thread(bind(&LaxRobot::motor_controller_worker, this));

  return true;
}

bool LaxRobot::stop()
{
  debug("Stopping the Lax Robot");

  flag_mutex_.lock();
  done_ = true;
  flag_mutex_.unlock();

  camera_thread_.join();
  motor_controller_thread_.join();

  flag_mutex_.lock();
  running_ = false;
  flag_mutex_.unlock();

  return true;
}

bool LaxRobot::ok()
{
  boost::unique_lock<boost::mutex> flag_lock(flag_mutex_);
  return ok_;
}

bool LaxRobot::done()
{
  boost::unique_lock<boost::mutex> flag_lock(flag_mutex_);
  return done_;
}

bool LaxRobot::running()
{
  boost::unique_lock<boost::mutex> flag_lock(flag_mutex_);
  return running_;
}
