//
// Created by sushant on 10/15/19.
//

#include "motor.hpp"
using namespace smart_drive_duo30;

motor::motor(int max_speed, int pwm_pin, int direction_pin) {
  this->max_speed = max_speed;
  this->pwm_pin = pwm_pin;
  this->direction_pin = direction_pin;
  this->current_speed = 0;
  this->enabled = false;
}

motor::~motor() {
  if (enabled) {
    disable();
  }
}

void motor::enable() { enabled = true; }

void motor::disable() {
  enabled = false;
  pwmWrite(this->pwm_pin, 0);
}

void motor::set_speed_with_ramp(int value) {
  if (!enabled) {
    return;
  }
  if (current_speed == 0) {
    for (int i = 1; i < NUM_RAMP_STEPS; ++i) {
      this->set_speed(i * value / NUM_RAMP_STEPS);
      boost::this_thread::sleep(
          boost::posix_time::microseconds(RAMP_STEP_TIME_US));
    }
  }
  this->set_speed(value);
}

void motor::set_speed(int value) {
  if (!enabled) {
    return;
  }
  if (value < 0) {
    this->direction = BACKWARD;
    value = -value;
  } else {
    this->direction = FORWARD;
  }
  if (value > this->max_speed) {
    value = this->max_speed;
  }
  current_speed = value;
  digitalWrite(this->direction_pin, (int)this->direction);
  pwmWrite(this->pwm_pin, value);
}
