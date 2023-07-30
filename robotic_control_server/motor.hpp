//
// Created by sushant on 10/15/19.
//

#ifndef SMART_DRIVE_DUO30_MOTOR_CONTROLLER_MOTOR_HPP
#define SMART_DRIVE_DUO30_MOTOR_CONTROLLER_MOTOR_HPP

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <wiringPi.h>

namespace smart_drive_duo30 {

#define RAMP_STEP_TIME_US 10
#define NUM_RAMP_STEPS 500

class motor {
public:
  motor(int max_speed, int pwm_pin, int direction_pin);

  ~motor();

  void enable();

  void disable();

  void set_speed_with_ramp(int value);
  void set_speed(int value);

protected:
  int max_speed;
  int pwm_pin;
  int direction_pin;
  int current_speed;
  bool enabled;

  enum direction_type { FORWARD = 0, BACKWARD = 1 };
  direction_type direction;
};
} // namespace smart_drive_duo30

#endif // SMART_DRIVE_DUO30_MOTOR_CONTROLLER_MOTOR_HPP
