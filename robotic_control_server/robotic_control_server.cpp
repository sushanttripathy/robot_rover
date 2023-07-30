//
// Created by sushant on 7/21/19.
//
#include "robotic_control_server.hpp"

using namespace robotic_control_works;

robotic_control_server::robotic_control_server(std::string server_ip,
                                               int server_port,
                                               std::vector<int> &camera_indices)
    : thread_pool(NUM_WORKER_THREADS), socket(server_ip, server_port, 1),
      internal_communication_socket(INTERNAL_COMMUNICATION_IP,
                                    INTERNAL_COMMUNICATION_PORT, 1) {
  this->server_ip = server_ip;
  this->server_port = server_port;
  running = true;

  camera_fps = DEFAULT_CAMERA_FPS;

  this->camera_indices = camera_indices;
  for (auto camera_index : camera_indices) {
    camera_image_maps.insert(std::pair<int, std::vector<unsigned char>>(
        camera_index, std::vector<unsigned char>()));
    thread_pool.add_work(
        boost::bind(&robotic_control_server::timed_capture_from_camera, this,
                    camera_index, (int)(1000.0 / (float)camera_fps)));
  }

  motor_controller = smart_drive_duo30::motor_controller::get_instance();
  motor_controller->enable();

  thread_pool.add_work(boost::bind(
      &robotic_control_server::check_for_and_process_requests, this));

  thread_pool.add_work(boost::bind(
      &robotic_control_server::process_requests_on_internal_socket, this));
}

robotic_control_server::~robotic_control_server() {
  running = false;
  thread_pool.join_all();
}

bool robotic_control_server::is_running() { return running; }

void robotic_control_server::send_camera_frame(int camera_index,
                                               ServerResponse &response) {
  camera_lock.lock();
  if (!camera_image_maps.empty() &&
      camera_image_maps.find(camera_index) != camera_image_maps.end()) {
    // ServerResponse response;
    response.set_success(true);

    response.set_response_type(ServerResponse_ResponseType_IMAGE);
    if (camera_index == RGB_CAMERA_INDEX) {
      response.mutable_image_data()->set_rgb_image(
          camera_image_maps[camera_index].data(),
          camera_image_maps[camera_index].size());
    }
    if (camera_index == DEPTH_CAMERA_INDEX) {
      response.mutable_image_data()->set_depth_image(
          camera_image_maps[camera_index].data(),
          camera_image_maps[camera_index].size());
    }
    camera_lock.unlock();
    std::string serialized_string = response.SerializeAsString();
    zmq::const_buffer send_buf(serialized_string.data(),
                               serialized_string.size());
    socket_lock.lock();
    socket.send(send_buf, static_cast<zmq::send_flags>(0));
    socket_lock.unlock();

#ifdef IS_DEBUG_ENABLED
    std::cout << "Sent a response of size : " << send_buf.size() << std::endl;
#endif
  } else {
    camera_lock.unlock();
    send_error_message("READ_CAMERA request failed", "Camera index not found!",
                       response);
  }
}

void robotic_control_server::set_motor_speeds(int motor1_speed,
                                              int motor2_speed) {
  motor_controller->set_speeds(motor1_speed, motor2_speed);
}

void robotic_control_server::send_error_message(const char *message_summary,
                                                const char *message_details,
                                                ServerResponse &response) {
  response.set_success(false);
  response.set_response_type(ServerResponse_ResponseType_ERROR);
  response.set_summary_message(message_summary);
  response.set_detailed_message(message_details);
  std::string serialized_string = response.SerializeAsString();
  zmq::const_buffer send_buf(serialized_string.data(),
                             serialized_string.size());
  socket_lock.lock();
  socket.send(send_buf, static_cast<zmq::send_flags>(0));
  socket_lock.unlock();
}

void robotic_control_server::send_info_message(const char *message_summary,
                                               const char *message_details,
                                               ServerResponse &response) {
  response.set_success(true);
  response.set_response_type(ServerResponse_ResponseType_INFORMATION);
  response.set_summary_message(message_summary);
  response.set_detailed_message(message_details);
  std::string serialized_string = response.SerializeAsString();
  zmq::const_buffer send_buf(serialized_string.data(),
                             serialized_string.size());
  socket_lock.lock();
  socket.send(send_buf, static_cast<zmq::send_flags>(0));
  socket_lock.unlock();
}

void robotic_control_server::send_error_message_internal(
    const char *message_summary, const char *message_details) {
  ServerResponse response;
  response.set_success(false);
  response.set_response_type(ServerResponse_ResponseType_ERROR);
  response.set_summary_message(message_summary);
  response.set_detailed_message(message_details);
  std::string serialized_string = response.SerializeAsString();
  zmq::const_buffer send_buf(serialized_string.data(),
                             serialized_string.size());
  internal_socket_lock.lock();
  internal_communication_socket.send(send_buf, static_cast<zmq::send_flags>(0));
  internal_socket_lock.unlock();
}

void robotic_control_server::send_info_message_internal(
    const char *message_summary, const char *message_details) {
  ServerResponse response;
  response.set_success(true);
  response.set_response_type(ServerResponse_ResponseType_INFORMATION);
  response.set_summary_message(message_summary);
  response.set_detailed_message(message_details);
  std::string serialized_string = response.SerializeAsString();
  zmq::const_buffer send_buf(serialized_string.data(),
                             serialized_string.size());
  internal_socket_lock.lock();
  internal_communication_socket.send(send_buf, static_cast<zmq::send_flags>(0));
  internal_socket_lock.unlock();
}

void robotic_control_server::check_for_and_process_requests() {
  zmq::message_t request;
  while (running) {
    if (socket.recv(request, static_cast<zmq::recv_flags>(0))) {
      // if (socket.recv(request, static_cast<zmq::recv_flags>(ZMQ_NOBLOCK))) {
      // process the request message
#ifdef IS_DEBUG_ENABLED
      std::cout << "Received a request! Size : " << request.size() << " bytes"
                << std::endl;
#endif
      try {
        ControlRequest parsed_request;
        ServerResponse response;

        parsed_request.ParseFromArray(request.data(), request.size());
        if (!parsed_request.has_request_type()) {
          send_error_message("Bad request", "Doesn't have request_type.",
                             response);
          boost::this_thread::sleep(
              boost::posix_time::milliseconds(SERVER_PROCESS_SLEEP_TIME_MS));
          continue;
        }
        if (parsed_request.has_request_timestamp()) {
          response.set_request_timestamp(parsed_request.request_timestamp());
        }
        if (parsed_request.has_request_uuid()) {
          response.set_request_uuid(parsed_request.request_uuid());
        }
        switch (parsed_request.request_type()) {
        case ControlRequest_RequestType_READ_CAMERA:
          process_read_camera_request(parsed_request, response);
          break;
        case ControlRequest_RequestType_WRITE_MOTOR_CONTROLLER:
          process_write_motor_request(parsed_request, response);
          break;
        case ControlRequest_RequestType_READ_SENSORS_DATA:
          process_read_sensors_data_request(response);
          break;
        case ControlRequest_RequestType_READ_ALL_CAMERA_AND_SENSORS_DATA:
          process_read_all_camera_and_sensors_data_request(response);
          break;
        default:
          send_error_message(
              "Unknown request",
              "This version of the server doesn't understand this request.",
              response);
          break;
        }
      } catch (std::exception &e) {
        ServerResponse response;
        send_error_message("Unknown error", "No idea what went wrong!",
                           response);
      }
    }

    //    boost::this_thread::sleep(
    //        boost::posix_time::milliseconds(SERVER_PROCESS_SLEEP_TIME_MS));
  }
}

void robotic_control_server::timed_capture_from_camera(int camera_index,
                                                       int wait_time_ms) {
  std::vector<int> params_jpeg;
  params_jpeg.push_back(cv::IMWRITE_JPEG_QUALITY);
  params_jpeg.push_back(JPEG_IMAGE_QUALITY);
  std::vector<int> params_png;
  params_png.push_back(cv::IMWRITE_PNG_COMPRESSION);
  params_png.push_back(PNG_COMPRESSION_LEVEL);
  std::vector<int> params_empty;
  while (running) {
    std::vector<unsigned char> output_buffer;
    cv_works::camera_factory *camera = cv_works::camera_factory::get_instance();
    if (camera_index == RGB_CAMERA_INDEX) {
      camera->read_frame_in_format(camera_index, output_buffer,
                                   cv_works::FORMAT_JPEG, params_jpeg);
    } else {
      camera->read_frame_in_format(
          camera_index, output_buffer, cv_works::FORMAT_RAW, params_empty, 1,
          &robotic_control_server::
              process_raw_stereo_image_to_three_channel_image);
      //      camera->read_frame_in_format(camera_index, output_buffer,
      //                                   cv_works::FORMAT_PNG_GRAYSCALE,
      //                                   params_empty, 3);
      //      camera->read_frame_in_format(camera_index, output_buffer,
      //                                   cv_works::FORMAT_TIF_GRAYSCALE,
      //                                   params_empty, 3);
    }
    camera_lock.lock();
    camera_image_maps[camera_index].swap(output_buffer);
    camera_lock.unlock();
    boost::this_thread::sleep(boost::posix_time::milliseconds(wait_time_ms));
  }
}

void robotic_control_server::process_requests_on_internal_socket() {
  zmq::message_t request;
  while (running) {
    if (internal_communication_socket.recv(
            request, static_cast<zmq::recv_flags>(ZMQ_NOBLOCK))) {
      try {
        ControlRequest parsed_request;
        parsed_request.ParseFromArray(request.data(), request.size());
        if (!parsed_request.has_request_type()) {
          send_error_message_internal("Bad request",
                                      "Doesn't have request_type.");
          boost::this_thread::sleep(
              boost::posix_time::milliseconds(SERVER_PROCESS_SLEEP_TIME_MS));
          continue;
        }
        switch (parsed_request.request_type()) {
        case ControlRequest_RequestType_WRITE_SENSORS_DATA:
          process_write_sensors_data_request(parsed_request);
          break;
        default:
          send_error_message_internal(
              "Bad request",
              "This request type is not allowed on internal socket.");
          break;
        }
      } catch (std::exception &e) {
        send_error_message_internal("Unknown error",
                                    "No idea what went wrong!");
      }
    }
    boost::this_thread::sleep(
        boost::posix_time::milliseconds(SERVER_PROCESS_SLEEP_TIME_MS));
  }
}

void robotic_control_server::process_read_sensors_data_request(
    ServerResponse &response) {
  response.set_success(true);
  response.set_response_type(ServerResponse_ResponseType_SENSORS_DATA);
  sensors_lock.lock();
  response.mutable_sensors_data()->CopyFrom(sensors_data);
  sensors_lock.unlock();
  std::string serialized_string = response.SerializeAsString();
  zmq::const_buffer send_buf(serialized_string.data(),
                             serialized_string.size());
  socket_lock.lock();
  socket.send(send_buf, static_cast<zmq::send_flags>(0));
  socket_lock.unlock();
}

void robotic_control_server::process_read_all_camera_and_sensors_data_request(
    ServerResponse &response) {
  response.set_response_type(
      ServerResponse_ResponseType_IMAGES_AND_SENSORS_DATA);
  camera_lock.lock();
  if (!camera_image_maps.empty() &&
      camera_image_maps.find(RGB_CAMERA_INDEX) != camera_image_maps.end() &&
      camera_image_maps.find(DEPTH_CAMERA_INDEX) != camera_image_maps.end()) {
    // ServerResponse response;

    response.mutable_image_data()->set_rgb_image(
        camera_image_maps[RGB_CAMERA_INDEX].data(),
        camera_image_maps[RGB_CAMERA_INDEX].size());
    response.mutable_image_data()->set_depth_image(
        camera_image_maps[DEPTH_CAMERA_INDEX].data(),
        camera_image_maps[DEPTH_CAMERA_INDEX].size());
  }
  camera_lock.unlock();
  sensors_lock.lock();
  response.mutable_sensors_data()->CopyFrom(sensors_data);
  sensors_lock.unlock();

  response.set_success(true);

  std::string serialized_string = response.SerializeAsString();
  zmq::const_buffer send_buf(serialized_string.data(),
                             serialized_string.size());
  socket_lock.lock();
  socket.send(send_buf, static_cast<zmq::send_flags>(0));
  socket_lock.unlock();
}

void robotic_control_server::process_read_camera_request(
    ControlRequest &request, ServerResponse &response) {
  if (!request.has_camera_type()) {
    send_error_message("READ_CAMERA request failed",
                       "No specified camera type.", response);
    return;
  }
  int camera_index;

  switch (request.camera_type()) {
  case ControlRequest_CameraType_RGB_CAMERA:
    camera_index = RGB_CAMERA_INDEX;
    break;
  case ControlRequest_CameraType_DEPTH_CAMERA:
    camera_index = DEPTH_CAMERA_INDEX;
    break;
  default:
    camera_index = RGB_CAMERA_INDEX;
    break;
  }
  send_camera_frame(camera_index, response);
}

void robotic_control_server::process_write_motor_request(
    ControlRequest &request, ServerResponse &response) {
  if (!(request.has_motor_speeds() &&
        request.motor_speeds().has_motor_1_speed() &&
        request.motor_speeds().has_motor_2_speed())) {
    send_error_message("WRITE_MOTOR_CONTROLLER request failed",
                       "Doesn't have both motor parameters.", response);
    return;
  }
  int motor1_speed = request.motor_speeds().motor_1_speed();
  int motor2_speed = request.motor_speeds().motor_2_speed();
  set_motor_speeds(motor1_speed, motor2_speed);
  send_info_message("WRITE_MOTOR_CONTROLLER succeeded", "", response);
}

void robotic_control_server::process_write_sensors_data_request(
    ControlRequest &request) {
  if (!request.has_sensors_data()) {
    send_error_message_internal("Bad request",
                                "This request doesn't have a payload.");
    return;
  }
  sensors_lock.lock();
  sensors_data.CopyFrom(request.sensors_data());
  sensors_lock.unlock();
  send_info_message_internal("Sensors recorded",
                             "Successfully recorded sensors data.");
}

cv::Mat robotic_control_server::process_raw_stereo_image_to_disparity_map(
    cv::Mat &frame) {
  if (frame.empty()) {
    return cv::Mat();
  }
  cv::Mat left_right[2];
  cv::split(frame, left_right);
  cv::Mat left;
  cv::Mat right;
  cv::cvtColor(left_right[0], right, cv::COLOR_BayerGB2GRAY);
  cv::cvtColor(left_right[1], left, cv::COLOR_BayerGB2GRAY);
  cv::Mat resized_left;
  cv::Mat resized_right;
  cv::resize(right, resized_right, cv::Size(), 0.25, 0.25);
  cv::resize(left, resized_left, cv::Size(), 0.25, 0.25);
  cv::Mat equalized_left;
  cv::Mat equalized_right;

  cv::equalizeHist(resized_left, equalized_left);
  cv::equalizeHist(resized_right, equalized_right);
  cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(64, 5);
  cv::Mat disparity;
  bm->compute(equalized_left, equalized_right, disparity);
  return disparity;
}

cv::Mat robotic_control_server::process_raw_stereo_image_to_three_channel_image(
    cv::Mat &frame) {
  if (frame.empty()) {
    return cv::Mat();
  }
  cv::Mat left_right[2];
  cv::split(frame, left_right);
  cv::Mat left;
  cv::Mat right;
  cv::cvtColor(left_right[0], right, cv::COLOR_BayerGB2GRAY);
  cv::cvtColor(left_right[1], left, cv::COLOR_BayerGB2GRAY);
  cv::Mat resized_left;
  cv::Mat resized_right;
  cv::resize(right, resized_right, cv::Size(), 0.5, 0.5);
  cv::resize(left, resized_left, cv::Size(), 0.5, 0.5);

  cv::Mat empty =
      cv::Mat::zeros(cv::Size(resized_left.cols, resized_left.rows), CV_8UC1);
  std::vector<cv::Mat> channels;
  channels.push_back(resized_left);
  channels.push_back(empty);
  channels.push_back(resized_right);
  cv::Mat output;
  cv::merge(channels, output);

  return output;
}

cv::Mat robotic_control_server::
    process_raw_stereo_image_to_three_channel_image_with_side_by_side_layout(
        cv::Mat &frame) {
  if (frame.empty()) {
    return cv::Mat();
  }
  cv::Mat left_right[2];
  cv::split(frame, left_right);
  cv::Mat left;
  cv::Mat right;
  cv::cvtColor(left_right[0], right, cv::COLOR_BayerGB2BGR);
  cv::cvtColor(left_right[1], left, cv::COLOR_BayerGB2BGR);
  cv::Size size_left = left.size();
  cv::Size size_right = right.size();
  cv::Mat joined_image(std::max(size_left.height, size_right.height),
                       size_left.width + size_right.width, CV_8UC3);
  cv::Mat left_roi(joined_image,
                   cv::Rect(0, 0, size_left.width, size_left.height));
  cv::Mat right_roi(joined_image, cv::Rect(size_left.width, 0, size_right.width,
                                           size_right.height));
  left.copyTo(left_roi);
  right.copyTo(right_roi);
  return joined_image;
}
