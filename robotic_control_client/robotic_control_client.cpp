//
// Created by sushant on 7/29/19.
//

#include "robotic_control_client.hpp"

using namespace robotic_control_works;

robotic_control_client::robotic_control_client(
    std::string host, int port, ::ControlRequest_CameraType camera_type)
    : socket(host, port, 2), thread_pool(NUM_WORKER_THREADS), joystick() {
  running = true;

  this->camera_type = camera_type;
  x = 0;
  y = 0;
  spin = 0;

  thread_pool.add_work(
      boost::bind(&robotic_control_client::capture_joystick_events, this));
  thread_pool.add_work(
      boost::bind(&robotic_control_client::process_joystick_events, this));
  thread_pool.add_work(boost::bind(
      &robotic_control_client::display_camera_and_sensors_output, this));
}

robotic_control_client::~robotic_control_client() { running = false; }

bool robotic_control_client::is_running() { return running; }

void robotic_control_client::display_camera_output() {
  while (running) {
    ControlRequest request;
    request.set_request_uuid(make_uuid());
    request.set_request_timestamp(get_utc_timestamp_ms());
    request.set_request_type(ControlRequest_RequestType_READ_CAMERA);
    request.set_camera_type(camera_type);

    std::string serialized_string = request.SerializeAsString();
    zmq::const_buffer send_buf(serialized_string.data(),
                               serialized_string.size());

    socket_lock.lock();
    socket.send(send_buf, static_cast<zmq::send_flags>(0));
    zmq::message_t reply;
    socket.recv(reply, static_cast<zmq::recv_flags>(0));
    //    std::cout << reply.size() << std::endl;
    //    std::cout << reply.str() << std::endl;
    socket_lock.unlock();

    if (!reply.size()) {
      boost::this_thread::sleep(
          boost::posix_time::milliseconds(SERVER_PROCESS_SLEEP_TIME_MS));
      continue;
    }

    ServerResponse response;
    response.ParsePartialFromArray(reply.data(), reply.size());
    if (!response.success() || !response.has_image_data()) {
      boost::this_thread::sleep(
          boost::posix_time::milliseconds(SERVER_PROCESS_SLEEP_TIME_MS));
      continue;
    }

    std::vector<unsigned char> data;
    if (response.image_data().has_rgb_image()) {
      data.insert(data.begin(), response.image_data().rgb_image().data(),
                  response.image_data().rgb_image().data() +
                      response.image_data().rgb_image().size());
    }
    if (response.image_data().has_depth_image()) {
      data.insert(data.begin(), response.image_data().depth_image().data(),
                  response.image_data().depth_image().data() +
                      response.image_data().depth_image().size());
    }

    cv::Mat image = cv::imdecode(data, cv::IMREAD_UNCHANGED);
    // display the result
    if (data.size() && running) {
      if (camera_type == ControlRequest_CameraType_DEPTH_CAMERA) {
        cv::Mat blurred_image(image.size(), CV_8UC1);
        cv::Mat morphed_image(image.size(), CV_8UC1);
        cv::medianBlur(image, blurred_image, 5);
        cv::morphologyEx(blurred_image, morphed_image, cv::MORPH_OPEN, 5);
        // cv::equalizeHist( morphed_image, equalized_image );
        ui_lock.lock();
        cv::imshow(DISPLAY_WINDOW_NAME, morphed_image);
        if ((char)cv::waitKey(1) == 'q') {
          running = false;
        }
        ui_lock.unlock();
      } else {
        ui_lock.lock();
        cv::imshow(DISPLAY_WINDOW_NAME, image);
        if ((char)cv::waitKey(1) == 'q') {
          running = false;
        }
        ui_lock.unlock();
      }
    }

    boost::this_thread::sleep(
        boost::posix_time::milliseconds(SERVER_PROCESS_SLEEP_TIME_MS));
  }
}

void robotic_control_client::capture_joystick_events() {
  while (running) {
    JoystickEvent event;
    if (!joystick.sample(&event)) {
      continue;
    }
    std::cout << event << std::endl;
    if (!(event.isAxis() || event.isButton())) {
      continue;
    }
    speed_lock.lock();
    if (event.isButton()) {
      if (event.number == 1) {
        spin = event.value * 1;
      }
      if (event.number == 3) {
        spin = event.value * -1;
      }
      if (event.number == 4) {
        y = 25000 * event.value;
      }
      if (event.number == 0) {
        y = -25000 * event.value;
      }
    }
    if (event.isAxis() && event.number < 4 && event.number > 1) {
      if (event.number % 2 == 0) {
        x = event.value;
      } else {
        y = -event.value;
      }
    }
    speed_lock.unlock();
    boost::this_thread::sleep(
        boost::posix_time::milliseconds(SERVER_PROCESS_SLEEP_TIME_MS));
  }
}

void robotic_control_client::process_joystick_events() {
  while (running) {
    speed_lock.lock();
    // std::cout << "(" << x << "," << y << ")" << std::endl;
    motor_speeds speeds = convert_joystick_input_to_motor_speeds(x, y, spin);
    speed_lock.unlock();

    ControlRequest request;
    request.set_request_uuid(make_uuid());
    request.set_request_timestamp(get_utc_timestamp_ms());
    request.set_request_type(ControlRequest_RequestType_WRITE_MOTOR_CONTROLLER);
    request.mutable_motor_speeds()->set_motor_1_speed(speeds.motor1_speed);
    request.mutable_motor_speeds()->set_motor_2_speed(speeds.motor2_speed);
    std::string serialized_string = request.SerializeAsString();

    zmq::const_buffer send_buf(serialized_string.data(),
                               serialized_string.size());

    socket_lock.lock();
    socket.send(send_buf, static_cast<zmq::send_flags>(0));
    zmq::message_t reply;
    socket.recv(reply, static_cast<zmq::recv_flags>(0));
    socket_lock.unlock();
    boost::this_thread::sleep(
        boost::posix_time::milliseconds(SERVER_PROCESS_SLEEP_TIME_MS * 5));
  }
}

motor_speeds robotic_control_client::convert_joystick_input_to_motor_speeds(
    int x_in, int y_in, int spin) {
  motor_speeds output;
  output.motor1_speed = 0; // left
  output.motor2_speed = 0; // right

  if (spin != 0) {
    output.motor1_speed = (int)-spin * MAX_MOTOR_SPEED; // left
    output.motor2_speed = (int)-spin * MAX_MOTOR_SPEED; // right
    return output;
  }

  if (x_in == 0) {
    // Forward or backwards speeds only.
    output.motor1_speed = (int)(((float)(-y_in) / (float)MAX_JOYSTICK_INPUT) *
                                (float)MAX_MOTOR_SPEED);
    output.motor2_speed = (int)(((float)(y_in) / (float)MAX_JOYSTICK_INPUT) *
                                (float)MAX_MOTOR_SPEED);
    return output;
  }
  // Angle of the motion vector.
  float angle = abs(fast_arctan(y_in, x_in)) / PI_2;
  float magnitude = (int)((fast_square_root(x_in * x_in + y_in * y_in) /
                           ((float)MAX_JOYSTICK_INPUT * 1.4142)) *
                          (float)MAX_MOTOR_SPEED);

  std::cout << " Angle : " << angle << " Mag : " << magnitude << std::endl;

  if (x_in > 0) {
    if (y_in >= 0) {
      output.motor1_speed = -magnitude;
      output.motor2_speed = (int)((float)magnitude * angle);
      return output;
    } else {
      output.motor1_speed = magnitude;
      output.motor2_speed = -(int)((float)magnitude * angle);
      return output;
    }
  } else {
    if (y_in >= 0) {
      output.motor2_speed = magnitude;
      output.motor1_speed = -(int)((float)magnitude * angle);
      return output;
    } else {
      output.motor2_speed = -magnitude;
      output.motor1_speed = (int)((float)magnitude * angle);
      return output;
    }
  }
}

float robotic_control_client::fast_arctan(int perpendicular, int base) {
  if (base == 0) {
    return PI_2;
  }
  float ratio = ((float)perpendicular) / ((float)base);
  return PI_4 * ratio -
         ratio * (fabs(ratio) - 1) * (0.2447 + 0.0663 * fabs(ratio));
}

float robotic_control_client::fast_square_root(float input) {
  const float input_half = 0.5f * input;
  union // get bits for floating value
  {
    float x_;
    int i_;
  } u;
  u.x_ = input;
  u.i_ = SQRT_MAGIC_F - (u.i_ >> 1); // gives initial guess y0
  return input * u.x_ *
         (1.5f - input_half * u.x_ *
                     u.x_); // Newton step, repeating increases accuracy
}

void robotic_control_client::display_camera_and_sensors_output() {
  while (running) {
    ControlRequest request;
    request.set_request_uuid(make_uuid());
    request.set_request_timestamp(get_utc_timestamp_ms());
    request.set_request_type(
        ControlRequest_RequestType_READ_ALL_CAMERA_AND_SENSORS_DATA);

    std::string serialized_string = request.SerializeAsString();
    zmq::const_buffer send_buf(serialized_string.data(),
                               serialized_string.size());

    socket_lock.lock();
    socket.send(send_buf, static_cast<zmq::send_flags>(0));
    zmq::message_t reply;
    socket.recv(reply, static_cast<zmq::recv_flags>(0));
    socket_lock.unlock();

    if (!reply.size()) {
      boost::this_thread::sleep(
          boost::posix_time::milliseconds(SERVER_PROCESS_SLEEP_TIME_MS));
      continue;
    }
    std::cout << reply.size() << std::endl;
    ServerResponse response;
    response.ParsePartialFromArray(reply.data(), reply.size());
    if (!response.success() || !response.has_image_data() ||
        !response.has_sensors_data()) {
      boost::this_thread::sleep(
          boost::posix_time::milliseconds(SERVER_PROCESS_SLEEP_TIME_MS));
      continue;
    }

    // TODO: Move this outside the loop.
    std::vector<unsigned char> rgb_image_data;
    std::vector<unsigned char> depth_image_data;

    if (response.image_data().has_rgb_image()) {
      rgb_image_data.insert(rgb_image_data.begin(),
                            response.image_data().rgb_image().data(),
                            response.image_data().rgb_image().data() +
                                response.image_data().rgb_image().size());

    } else {
      continue;
    }
    if (response.image_data().has_depth_image()) {
      depth_image_data.insert(depth_image_data.begin(),
                              response.image_data().depth_image().data(),
                              response.image_data().depth_image().data() +
                                  response.image_data().depth_image().size());
    } else {
      continue;
    }

    cv::Mat rgb_image = cv::imdecode(rgb_image_data, cv::IMREAD_UNCHANGED);
    cv::Mat depth_image = cv::imdecode(depth_image_data, cv::IMREAD_UNCHANGED);

    cv::Mat blurred_depth_image(depth_image.size(), depth_image.type());
    cv::medianBlur(depth_image, blurred_depth_image, 5);
    cv::morphologyEx(blurred_depth_image, depth_image, cv::MORPH_OPEN, 5);

    cv::Mat resized_depth_image;
    float resize_ratio =
        std::min(1.0, std::min(double(rgb_image.size().width) /
                                   double(depth_image.size().width),
                               double(rgb_image.size().height) /
                                   double(depth_image.size().height)));
    cv::resize(depth_image, resized_depth_image, cv::Size(), resize_ratio,
               resize_ratio);
    cv::Mat resized_rgb_depth_image;

    if (resized_depth_image.type() == CV_8UC1) {
      cv::cvtColor(resized_depth_image, resized_rgb_depth_image,
                   cv::COLOR_GRAY2BGR);
    } else {
      resized_rgb_depth_image = resized_depth_image;
    }
    //    resized_rgb_depth_image.copyTo(rgb_image(cv::Rect(
    //        0, 0, resized_rgb_depth_image.cols,
    //        resized_rgb_depth_image.rows)));

    cv::Mat ui_image(rgb_image.size().height, rgb_image.size().width * 2,
                     CV_8UC3);

    cv::Mat depth_roi(ui_image,
                      cv::Rect(0, 0, resized_rgb_depth_image.size().width,
                               resized_rgb_depth_image.size().height));
    cv::Mat rgb_roi(ui_image,
                    cv::Rect(rgb_image.size().width, 0, rgb_image.size().width,
                             rgb_image.size().height));
    resized_rgb_depth_image.copyTo(depth_roi);
    rgb_image.copyTo(rgb_roi);
    ui_lock.lock();
    cv::imshow(DISPLAY_WINDOW_NAME, ui_image);
    if ((char)cv::waitKey(1) == 'q') {
      running = false;
    }
    ui_lock.unlock();

    //    boost::this_thread::sleep(
    //        boost::posix_time::milliseconds(SERVER_PROCESS_SLEEP_TIME_MS));
  }
}

long robotic_control_client::get_utc_timestamp_ms() {
  boost::posix_time::ptime current_date_microseconds =
      boost::posix_time::microsec_clock::local_time();
  long milliseconds =
      current_date_microseconds.time_of_day().total_milliseconds();
  return milliseconds;
}

std::string robotic_control_client::make_uuid() {
  boost::uuids::uuid uuid = uuid_generator();
  std::stringstream string_stream;
  string_stream << uuid;
  return string_stream.str();
}
