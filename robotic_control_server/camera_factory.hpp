//
// Created by sushant on 7/28/19.
//

#ifndef ZMQ_IMAGE_SERVER_CAMERA_FACTORY_HPP
#define ZMQ_IMAGE_SERVER_CAMERA_FACTORY_HPP
#include <boost/thread/mutex.hpp>
#include <map>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace cv_works {
enum format_t {
  FORMAT_NONE,
  FORMAT_JPEG,
  FORMAT_PNG,
  FORMAT_PNG_GRAYSCALE,
  FORMAT_TIF_GRAYSCALE,
  FORMAT_RAW
};

class camera_factory {
public:
  static camera_factory *get_instance();
  void read_frame_in_format(int camera_index, std::vector<unsigned char> &data,
                            format_t format);
  void read_frame_in_format(
      int camera_index, std::vector<unsigned char> &data, format_t format,
      const std::vector<int> &params, int average_over_n_frames = 1,
      std::function<cv::Mat(cv::Mat &)> preprocessing_function = nullptr);

  ~camera_factory();

private:
  static bool is_instantiated;
  static camera_factory *instance;
  static boost::mutex *main_mutex;
  //  std::map<int, int> frame_grab_counts;
  std::map<int, int> camera_index_to_array_index;
  std::vector<boost::mutex *> camera_mutexes;
  std::vector<cv::VideoCapture *> camera_interfaces;
  int count;

  camera_factory();

  static void read_frame(
      boost::mutex *camera_mutex, cv::VideoCapture *capture_interface,
      std::vector<unsigned char> &data, format_t format,
      const std::vector<int> &params, int average_over_n_frames = 1,
      std::function<cv::Mat(cv::Mat &)> preprocessing_function = nullptr);
  static cv::Mat equalizeIntensity(cv::Mat &input_image);
};
} // namespace cv_works

#endif // ZMQ_IMAGE_SERVER_CAMERA_FACTORY_HPP
