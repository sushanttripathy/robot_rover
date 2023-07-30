//
// Created by sushant on 7/29/19.
//

#ifndef ROBOTIC_CONTROL_CLIENT_ROBOTIC_CONTROL_CLIENT_HPP
#define ROBOTIC_CONTROL_CLIENT_ROBOTIC_CONTROL_CLIENT_HPP

#include "joystick.hpp"
#include "proto/control_request.pb.h"
#include "proto/server_response.pb.h"
#include "thread_pool.hpp"
#include "zmq_socket.hpp"
#include <boost/thread/mutex.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <map>
#include <math.h>
#include <opencv2/opencv.hpp>

namespace robotic_control_works {
#define NUM_WORKER_THREADS 3
#define DISPLAY_WINDOW_NAME "blank"
#define SERVER_PROCESS_SLEEP_TIME_MS 10
#define MAX_MOTOR_SPEED 480
#define MAX_JOYSTICK_INPUT 32767
#define PI_4 0.78539816339
#define PI_2 1.57079632679
#define SQRT_MAGIC_F 0x5f3759df
//#define RESPONSE_TIME_S 0.1
//#define AXLE_LENGTH_CM 25.0
//#define RADIUS_WHEEL_CM 6.0
//#define SPEED_CM_PER_S_TO_AU 4.857

struct motor_speeds {
  int motor1_speed;
  int motor2_speed;
};

class robotic_control_client {
public:
  robotic_control_client(std::string host, int port,
                         ::ControlRequest_CameraType camera_type);
  ~robotic_control_client();
  bool is_running();

private:
  ::ControlRequest_CameraType camera_type;
  zmq_sockets::zmq_client_socket socket;
  thread_works::thread_pool thread_pool;
  Joystick joystick;
  std::atomic_bool running;

  int x;
  int y;
  int spin;

  boost::mutex speed_lock;
  boost::mutex socket_lock;
  boost::mutex ui_lock;
  boost::uuids::random_generator uuid_generator;

  long get_utc_timestamp_ms();
  std::string make_uuid();

  void display_camera_and_sensors_output();
  void display_camera_output();
  void capture_joystick_events();
  void process_joystick_events();
  motor_speeds convert_joystick_input_to_motor_speeds(int x, int y,
                                                      int spin = 0);
  float fast_square_root(float input);
  float fast_arctan(int perpendicular, int base);
};
} // namespace robotic_control_works

#endif // ROBOTIC_CONTROL_CLIENT_ROBOTIC_CONTROL_CLIENT_HPP
