//
// Created by sushant on 10/15/19.
//

#ifndef SMART_DRIVE_DUO30_MOTOR_CONTROLLER_MOTOR_CONTROLLER_HPP
#define SMART_DRIVE_DUO30_MOTOR_CONTROLLER_MOTOR_CONTROLLER_HPP

#include "motor.hpp"
#include "thread_pool.hpp"

#define MAX_SPEED 480
#define NUM_MOTOR_THREADS 2

#include <wiringPi.h>

namespace smart_drive_duo30 {
class motor_controller {
private:
  static bool instantiated;

  motor *motor1;
  motor *motor2;
  thread_work::thread_pool thread_pool;

  motor_controller();

  void init_io();

  static motor_controller *inited_instance;

public:
  static motor_controller *get_instance();

  ~motor_controller();

  void set_speeds(int motor1_speed, int motor2_speed);

  void set_motor_speed(int motor_index, int speed);

  void enable();

  void disable();
};
} // namespace smart_drive_duo30

#endif // SMART_DRIVE_DUO30_MOTOR_CONTROLLER_MOTOR_CONTROLLER_HPP
