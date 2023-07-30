//
// Created by sushant on 7/21/19.
//

#ifndef ZMQ_IMAGE_SERVER_ROBOTIC_CONTROL_SERVER_HPP
#define ZMQ_IMAGE_SERVER_ROBOTIC_CONTROL_SERVER_HPP

#include "camera_factory.hpp"
#include "motor_controller.hpp"
#include "proto/control_request.pb.h"
#include "proto/server_response.pb.h"
#include "thread_pool.hpp"
#include "timer.hpp"
#include "zmq_socket.hpp"
#include <atomic>
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <map>
#include <string>
#include <vector>
#include <zmq.hpp>
//#define IS_DEBUG_ENABLED

namespace robotic_control_works {
#define DEFAULT_CAMERA_FPS 25
#define NUM_WORKER_THREADS 4
#define SERVER_PROCESS_SLEEP_TIME_MS 10
#define MAIN_LOOP_SLEEP_TIME_MS 500
#define RGB_CAMERA_INDEX 2
#define DEPTH_CAMERA_INDEX 0
#define JPEG_IMAGE_QUALITY 90
#define PNG_COMPRESSION_LEVEL 9
#define INTERNAL_COMMUNICATION_IP "127.0.0.1"
#define INTERNAL_COMMUNICATION_PORT 2015

class robotic_control_server {
public:
  robotic_control_server(std::string server_ip, int server_port,
                         std::vector<int> &camera_indices);
  ~robotic_control_server();
  bool is_running();

private:
  std::string server_ip;
  int server_port;

  std::atomic_bool running;
  zmq_sockets::zmq_server_socket socket;
  zmq_sockets::zmq_server_socket internal_communication_socket;
  int camera_fps;

  std::vector<int> camera_indices;
  std::map<int, std::vector<unsigned char>> camera_image_maps;

  boost::mutex camera_lock;
  boost::mutex sensors_lock;
  boost::mutex socket_lock;
  boost::mutex internal_socket_lock;
  thread_work::thread_pool thread_pool;
  smart_drive_duo30::motor_controller *motor_controller;

  SensorsData sensors_data;

  void check_for_and_process_requests();

  void timed_capture_from_camera(int camera_index, int wait_time_ms);
  void send_camera_frame(int camera_index, ServerResponse &response);
  void
  process_read_all_camera_and_sensors_data_request(ServerResponse &response);

  void process_read_camera_request(ControlRequest &request,
                                   ServerResponse &response);
  void process_write_motor_request(ControlRequest &request,
                                   ServerResponse &response);

  void set_motor_speeds(int motor1_speed, int motor2_speed);
  void send_error_message(const char *message_summary,
                          const char *message_details,
                          ServerResponse &response);
  void send_info_message(const char *message_summary,
                         const char *message_details, ServerResponse &response);

  void process_read_sensors_data_request(ServerResponse &response);
  void process_write_sensors_data_request(ControlRequest &request);

  void send_error_message_internal(const char *message_summary,
                                   const char *message_details);
  void send_info_message_internal(const char *message_summary,
                                  const char *message_details);
  void process_requests_on_internal_socket();

  static cv::Mat process_raw_stereo_image_to_disparity_map(cv::Mat &frame);

  // This method processes the stereo image into a 3 channel iamge, where
  // channels 0 and 2 contain left and right (monochromatic) stereo images
  // respectively. Channel 1 is empty.
  static cv::Mat
  process_raw_stereo_image_to_three_channel_image(cv::Mat &frame);

  static cv::Mat
  process_raw_stereo_image_to_three_channel_image_with_side_by_side_layout(
      cv::Mat &frame);
};
}; // namespace robotic_control_works

#endif // ZMQ_IMAGE_SERVER_ROBOTIC_CONTROL_SERVER_HPP
