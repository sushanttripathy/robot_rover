//
// Created by sushant on 7/28/19.
//

#include "camera_factory.hpp"
using namespace cv_works;

bool camera_factory::is_instantiated = false;
camera_factory *camera_factory::instance = nullptr;
boost::mutex *camera_factory::main_mutex = new boost::mutex;

namespace {
cv::Mat get_mean_image(const std::vector<cv::Mat> &images) {
  if (images.empty())
    return cv::Mat();

  // Create a 0 initialized image to use as accumulator
  cv::Mat m(images[0].rows, images[0].cols, CV_32FC3);
  m.setTo(cv::Scalar(0, 0, 0, 0));

  // Use a temp image to hold the conversion of each input image to CV_32FC3
  // This will be allocated just the first time, since all your images have
  // the same size.
  cv::Mat temp;
  for (int i = 0; i < images.size(); ++i) {
    // Convert the input images to CV_64FC3 ...
    images[i].convertTo(temp, CV_32FC3);

    // ... so you can accumulate
    m += temp;
  }
  // Convert back to CV_8UC3 type, applying the division to get the actual mean
  m.convertTo(m, CV_8U, 1. / images.size());
  return m;
}

} // namespace

camera_factory *camera_factory::get_instance() {
  camera_factory::main_mutex->lock();
  if (camera_factory::is_instantiated) {
    camera_factory::main_mutex->unlock();
    return camera_factory::instance;
  } else {
    camera_factory::instance = new camera_factory();
    camera_factory::is_instantiated = true;
    camera_factory::main_mutex->unlock();
    return camera_factory::instance;
  }
}

void camera_factory::read_frame_in_format(int camera_index,
                                          std::vector<unsigned char> &data,
                                          cv_works::format_t format) {
  std::vector<int> params;
  return read_frame_in_format(camera_index, data, format, params);
}
void camera_factory::read_frame_in_format(
    int camera_index, std::vector<unsigned char> &data,
    cv_works::format_t format, const std::vector<int> &params,
    int average_over_n_frames,
    std::function<cv::Mat(cv::Mat &)> preprocessing_function) {
  camera_factory::main_mutex->lock();
  if (camera_index_to_array_index.find(camera_index) ==
      camera_index_to_array_index.end()) {
    camera_index_to_array_index[camera_index] = count++;
    auto camera_mutex = new boost::mutex;
    camera_mutexes.push_back(camera_mutex);

    auto capture_interface = new cv::VideoCapture(camera_index);
    if (format == cv_works::FORMAT_RAW) {
      capture_interface->set(cv::CAP_PROP_CONVERT_RGB, 0);
    }
    camera_interfaces.push_back(capture_interface);
    camera_factory::main_mutex->unlock();
    read_frame(camera_mutex, capture_interface, data, format, params,
               average_over_n_frames, preprocessing_function);
  } else {
    int array_index = camera_index_to_array_index[camera_index];
    auto camera_mutex = camera_mutexes[array_index];
    auto capture_interface = camera_interfaces[array_index];
    camera_factory::main_mutex->unlock();
    //    if (format == cv_works::FORMAT_RAW) {
    //      capture_interface->set(cv::CAP_PROP_CONVERT_RGB, 0);
    //    }
    read_frame(camera_mutex, capture_interface, data, format, params,
               average_over_n_frames, preprocessing_function);
  }
}

void camera_factory::read_frame(
    boost::mutex *camera_mutex, cv::VideoCapture *capture_interface,
    std::vector<unsigned char> &data, cv_works::format_t format,
    const std::vector<int> &params, int average_over_n_frames,
    std::function<cv::Mat(cv::Mat &)> preprocessing_function) {
  if (camera_mutex == nullptr || capture_interface == nullptr) {
    return;
  }
  cv::Mat frame;
  camera_mutex->lock();
  if (average_over_n_frames <= 1 && capture_interface->isOpened()) {
    *capture_interface >> frame;
    camera_mutex->unlock();
  } else {
    int capture_count = 0;
    std::vector<cv::Mat> captured_frames;
    while (capture_count++ < average_over_n_frames &&
           capture_interface->isOpened()) {
      cv::Mat temp_frame;
      *capture_interface >> temp_frame;
      captured_frames.push_back(temp_frame);
    }
    camera_mutex->unlock();
    frame = ::get_mean_image(captured_frames);
    for (auto temp_frame = captured_frames.begin();
         temp_frame != captured_frames.end(); ++temp_frame) {
      temp_frame->release();
    }
  }
  if (frame.empty()) {
    frame.release();
    return;
  }
  switch (format) {
  case FORMAT_NONE:
    break;
  case FORMAT_RAW:
    if (frame.empty() || preprocessing_function == nullptr) {
      break;
    } else {
      cv::Mat new_frame = preprocessing_function(frame);
      if (params.empty()) {
        cv::imencode(".png", new_frame, data);
      } else {
        cv::imencode(".png", new_frame, data, params);
      }
      new_frame.release();
    }
    break;
  case FORMAT_JPEG:
    if (params.empty()) {
      cv::imencode(".jpg", equalizeIntensity(frame), data);
    } else {
      cv::imencode(".jpg", equalizeIntensity(frame), data, params);
    }
    break;
  case FORMAT_PNG:
    if (params.empty()) {
      cv::imencode(".png", equalizeIntensity(frame), data);
    } else {
      cv::imencode(".png", equalizeIntensity(frame), data, params);
    }
    break;
  case FORMAT_PNG_GRAYSCALE: {
    cv::Mat grayscale_image(frame.size(), CV_8UC1);
    cv::cvtColor(frame, grayscale_image, cv::COLOR_BGR2GRAY);
    if (params.empty()) {
      cv::imencode(".png", grayscale_image, data);
    } else {
      cv::imencode(".png", grayscale_image, data, params);
    }
    grayscale_image.release();
  } break;
  case FORMAT_TIF_GRAYSCALE: {
    cv::Mat grayscale_image(frame.size(), CV_8UC1);
    cv::cvtColor(frame, grayscale_image, cv::COLOR_BGR2GRAY);
    cv::imencode(".tif", grayscale_image, data);
    grayscale_image.release();
  } break;
  }
  frame.release();
}

camera_factory::camera_factory() { count = 0; }

camera_factory::~camera_factory() {}

cv::Mat camera_factory::equalizeIntensity(cv::Mat &input_image) {
  if (input_image.channels() >= 3) {
    cv::Mat ycrcb;

    cvtColor(input_image, ycrcb, cv::COLOR_BGR2YCrCb);

    std::vector<cv::Mat> channels;
    split(ycrcb, channels);

    equalizeHist(channels[0], channels[0]);

    cv::Mat result;
    merge(channels, ycrcb);

    cvtColor(ycrcb, result, cv::COLOR_YCrCb2BGR);

    return result;
  }
  return cv::Mat();
}
