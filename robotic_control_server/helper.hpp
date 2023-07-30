//
// Created by sushant on 10/26/19.
//

#ifndef ROBOTIC_CONTROL_SERVER_HELPER_HPP
#define ROBOTIC_CONTROL_SERVER_HELPER_HPP

#include <wiringPi.h>

void startup() {
  wiringPiSetupGpio();
  pinMode(16, OUTPUT);
  digitalWrite(16, 1);
}

void shutdown() { digitalWrite(16, 0); }

#endif // ROBOTIC_CONTROL_SERVER_HELPER_HPP
