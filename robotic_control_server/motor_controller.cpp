//
// Created by sushant on 10/15/19.
//

#include "motor_controller.hpp"
using namespace smart_drive_duo30;

bool motor_controller::instantiated;
motor_controller *motor_controller::inited_instance;

void motor_controller::init_io() {
  wiringPiSetupGpio();
  pinMode(12, PWM_OUTPUT);
  pinMode(13, PWM_OUTPUT);
  pwmSetMode(PWM_MODE_MS);
  pwmSetRange(MAX_SPEED);
  pwmSetClock(2);

  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
}

motor_controller::motor_controller() : thread_pool(NUM_MOTOR_THREADS) {
  this->init_io();

  this->motor1 = new motor(MAX_SPEED, 13, 6);
  this->motor2 = new motor(MAX_SPEED, 12, 5);
}

motor_controller::~motor_controller() {}

motor_controller *motor_controller::get_instance() {
  if (motor_controller::instantiated) {
    return motor_controller::inited_instance;
  }

  motor_controller::inited_instance = new motor_controller();
  motor_controller::instantiated = true;

  return motor_controller::inited_instance;
}

void motor_controller::enable() {
  this->motor1->enable();
  this->motor2->enable();
}

void motor_controller::disable() {
  this->motor1->disable();
  this->motor2->disable();
}

void motor_controller::set_speeds(int motor1_speed, int motor2_speed) {
  //    this->motor1->set_speed_with_ramp(motor1_speed);
  //    this->motor2->set_speed_with_ramp(motor2_speed);
  thread_pool.add_work(
      boost::bind(&motor::set_speed_with_ramp, this->motor1, motor1_speed));
  thread_pool.add_work(
      boost::bind(&motor::set_speed_with_ramp, this->motor2, motor2_speed));
}

void motor_controller::set_motor_speed(int motor_index, int speed) {
  switch (motor_index) {
  case 1:
    this->motor1->set_speed_with_ramp(speed);
  case 2:
    this->motor2->set_speed_with_ramp(speed);
  }
}
