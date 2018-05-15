#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <string.h>

#include <vector>

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <pixyhandle.hpp>
#include <lax_robot.h>

#define BLOCK_BUFFER_SIZE 10

using boost::bind;
using boost::mutex;
using boost::thread;
using boost::unique_lock;
using std::cin;
using std::cerr;
using std::endl;
using std::string;

const int LaxRobot::kHorizontalMaxPixel = 320;
const int LaxRobot::kVerticalMaxPixel = 240;
const double LaxRobot::kHorizontalFov = 75;
const double LaxRobot::kVerticalFov = 47;
const int LaxRobot::kMinBallArea = 10;
const int LaxRobot::kEncoderChannel = 1;
const int LaxRobot::kMotorChannel = 1;

LaxRobot::LaxRobot(
  const string motor_controller_port,
  const int motor_controller_baud_rate) :
  motor_controller_(motor_controller_port, motor_controller_baud_rate)
{
  debug("Constructing the Lax Robot");
  left_camera_index_ = 0;
  right_camera_index_ = 1;

  camera_positions_[0](0) = -12;
  camera_positions_[0](1) = 14;
  camera_positions_[0](2) = 0;

  camera_positions_[1](0) = 12;
  camera_positions_[1](1) = 14;
  camera_positions_[1](2) = 0;

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

  int blocks_copied[2];
  struct Block blocks[2][BLOCK_BUFFER_SIZE];
  double line_matrix[3][3];
  double inner_products[2][3];
  double solve_matrix[2][3];
  double points[2][3];
  double midpoint[3];
  int pos_array[2][2];
  double angle_array[2][2];
  int size_array[2];
  double lines[2][3];
  double t;
  double s;

  char print_buffer[501];
  
  while (active()) {
    for (int i = 0; i < 2; i++) {
      // Long waits are possible so check that the system is still active.
      while (!cameras_[i].blocks_are_new() && active());
    }

    if (active()) {
      for (int i = 0; i < 2; i++) {
        // Get blocks from Pixy //
        blocks_copied[i] = cameras_[i].get_blocks(BLOCK_BUFFER_SIZE, &(blocks[i][0]));

        if(blocks_copied[i] < 0) {
          debug("Invalid number of blocks copied");
          break;
        }

        //Check for maximum size block
        int max_size = 0;
        int max_index = 0;
        for (int index = 0; index != blocks_copied[i]; ++index) {
          int curr_size = blocks[i][index].width * blocks[i][index].height;
          if (curr_size > max_size) {
            max_size = curr_size;
            max_index = index;
          }
        }

        pos_array[i][0] = blocks[i][max_index].x;
        pos_array[i][1] = blocks[i][max_index].y;
        size_array[i] = max_size;
      }

      if (size_array[0] > kMinBallArea && size_array[1] > kMinBallArea)
      {
        CalculateAngles(kHorizontalFov,
                        kVerticalFov,
                        kHorizontalMaxPixel,
                        kVerticalMaxPixel,
                        pos_array,
                        angle_array);

        CalculateLineVectors(angle_array, lines);

        CreateLineMatrix(line_matrix,
                         lines,
                         camera_positions_[0](0),
                         camera_positions_[0](1),
                         camera_positions_[0](2),
                         camera_positions_[1](0),
                         camera_positions_[1](1),
                         camera_positions_[1](2));

        InnerProduct(line_matrix, lines[0], inner_products[0]);
        InnerProduct(line_matrix, lines[1], inner_products[1]);

        CreateSolveMatrix(solve_matrix, inner_products[0], inner_products[1]);

        RowReduce(solve_matrix);

        t = solve_matrix[0][2];
        s = solve_matrix[1][2];

        FindPoints(camera_positions_[0](0),
                   camera_positions_[0](1),
                   camera_positions_[0](2),
                   camera_positions_[1](0),
                   camera_positions_[1](1),
                   camera_positions_[1](2),
                   lines,
                   points[0],
                   points[1],
                   midpoint,
                   t,
                   s);

        snprintf(print_buffer,
                 500,
                 "3D Point Coordinate: X: %f, Y: %f, Z: %f.\n",
                 midpoint[0],
                 midpoint[1],
                 midpoint[2]);

        debug(print_buffer);

        position_mutex_.lock();  
        update_flag_ = true;
        arm_target_position_(0) = midpoint[0];
        arm_target_position_(1) = midpoint[1];
        position_mutex_.unlock();
      } 
    }
  }
}

void LaxRobot::motor_controller_worker()
{
  debug("Starting up the motor controller worker");

  bool updated = false;
  int encoder_count = 0;
  char print_buffer[101];

  while (active()) {
    position_mutex_.lock();  
    updated = update_flag_;
    if (update_flag_) {
      target_ = arm_target_position_;
      update_flag_ = false;
    }
    position_mutex_.unlock();

    /*if (updated) {
      encoder_count = 2500; 
      motor_controller_.motor_absolute_position_command(1, encoder_count);
    }*/

    if (updated) {
      float x = -arm_target_position_(1);
      float y = arm_target_position_(0);
      float theta = -atan2(y, x);

      /*if (theta * old_theta_ < 0) {
        if (fabs(old_theta_) > M_PI / 2) {
          if (theta > 0) {
            theta = 2 * M_PI - theta;
          } else {
            theta = - 2 * M_PI - theta;
          }
        } else {
          spin_counter_ += (theta > 0) ? 1 : -1;
        }
      }*/

      int counter = ((float) theta * 2500) /  M_PI + spin_counter_ * 5000;
      // motor_controller_.motor_absolute_position_command(1, counter);
      snprintf(print_buffer,
               100,
               "[x: %f, y: %f, theta: %f, counter: %d]",
               x, y, theta, counter);

      debug(print_buffer);
    }
  }
}

bool LaxRobot::configure_motor_controller()
{
  motor_controller_.open_serial();
  if (!motor_controller_.ok()) {
    debug("Could not open motor controller serial connection");
    return false;
  }

  debug_mutex_.lock();
  if (!motor_controller_.set_echo(false) ||
      !motor_controller_.set_encoder_high_count_limit(LaxRobot::kEncoderChannel, 10000) ||
      !motor_controller_.set_encoder_low_count_limit(LaxRobot::kEncoderChannel, -10000) ||
      !motor_controller_.set_encoder_usage(LaxRobot::kEncoderChannel, LaxRobot::kMotorChannel, RoboteqController::FEEDBACK) ||
      !motor_controller_.set_encoder_ppr(LaxRobot::kEncoderChannel, 1250) ||
      !motor_controller_.set_amp_limit(LaxRobot::kMotorChannel, 180) ||
      !motor_controller_.set_amp_trigger_level(LaxRobot::kMotorChannel, 150) ||
      !motor_controller_.set_amp_trigger_delay(LaxRobot::kMotorChannel, 1000) ||
      !motor_controller_.set_amp_trigger_action(LaxRobot::kMotorChannel, RoboteqController::SAFETY_STOP) ||
      !motor_controller_.set_proportional_gain(LaxRobot::kMotorChannel, 20) ||
      !motor_controller_.set_integral_gain(LaxRobot::kMotorChannel, 0) ||
      !motor_controller_.set_differential_gain(LaxRobot::kMotorChannel, 0) ||
      !motor_controller_.set_motor_acceleration(LaxRobot::kMotorChannel, 170) ||
      !motor_controller_.set_motor_decceleration(LaxRobot::kMotorChannel, 170) ||
      !motor_controller_.set_default_position_velocity(LaxRobot::kMotorChannel, 100) ||
      !motor_controller_.set_max_rpm(LaxRobot::kMotorChannel, 100) ||
      !motor_controller_.motor_position_mode_velocity_command(LaxRobot::kMotorChannel, 100) ||
      !motor_controller_.set_operating_mode(LaxRobot::kMotorChannel, RoboteqController::OPEN_LOOP_SPEED) ||
      !motor_controller_.encoder_count_command(LaxRobot::kMotorChannel, 0) ||
      !motor_controller_.set_operating_mode(LaxRobot::kMotorChannel, RoboteqController::CLOSED_LOOP_COUNT_POSITION)) {
    debug_mutex_.unlock();
    debug("Could not configure the motor controller with the default settings");
    return false;
  } else {
    debug_mutex_.unlock();
  }
  return true;
}

void LaxRobot::teardown_motor_controller()
{
}

bool LaxRobot::configure_cameras()
{
  if (PixyHandle::num_pixies_attached() != 2) {
    debug("Invalid number of pixies attached, expecting 2");
    return false;
  }

  for (int i = 0; i < 2; i++) {
    int camera_status = cameras_[i].init();
    if (camera_status != 0) {
      debug("Camera could not initialize");
      return false;
    }

    uint16_t major;
    uint16_t minor;
    uint16_t build;

    int firmware_version_check = cameras_[i].get_firmware_version(&major,
                                                                  &minor,
                                                                  &build);

    if (firmware_version_check) {
      debug("Firmware version check failed");
      return false;
    }
  }

  return true;
}

void LaxRobot::teardown_cameras()
{
}

bool LaxRobot::start()
{
  debug("Starting up the Lax Robot");

  /*if (!configure_motor_controller()) {
    debug("Failed to configure the motor controller");
    return false;
  }*/

  if (!configure_cameras()) {
    debug("Failed to configure the cameras");
    return false;
  }
  
  old_theta_ = 0;
  spin_counter_ = 0;

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

bool LaxRobot::active()
{
  boost::unique_lock<boost::mutex> flag_lock(flag_mutex_);
  return ok_ && !done_;
}
