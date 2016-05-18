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
  while (ok_ && !done_) {
    flag_mutex_.unlock();
    
    debug("Running motor controller loop"); 
    usleep(200000);

    flag_mutex_.lock();
  }
  flag_mutex_.unlock();
}

bool LaxRobot::start()
{
  debug("Starting up the Lax Robot");

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
