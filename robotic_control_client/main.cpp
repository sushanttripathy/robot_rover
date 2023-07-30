#include "robotic_control_client.hpp"
#include <iostream>
int main() {
  std::cout << "Hello, World!" << std::endl;
  std::string a;
  robotic_control_works::robotic_control_client *control_client =
      new robotic_control_works::robotic_control_client(
          "192.168.0.165", 2000, ControlRequest_CameraType_RGB_CAMERA);
  while (control_client->is_running()) {
    boost::this_thread::sleep(
        boost::posix_time::milliseconds(SERVER_PROCESS_SLEEP_TIME_MS * 10));
  }
  return 0;
}