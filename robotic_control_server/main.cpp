#include "daemonize.hpp"
#include "helper.hpp"
#include "robotic_control_server.hpp"
#include <iostream>

int main() {
  //  std::cout << "Starting robotic control server!" << std::endl;
  startup();
  // daemonize();
  std::vector<int> indices;
  indices.push_back(RGB_CAMERA_INDEX);
  indices.push_back(DEPTH_CAMERA_INDEX);
  robotic_control_works::robotic_control_server *control_server =
      new robotic_control_works::robotic_control_server("*", 2000, indices);
  //  std::cout << "Server online! Press 'q' to quit!" << std::endl;
  while (control_server->is_running()) {
    //    if (std::cin.get() == 'q') {
    //      std::cout << "Breaking " << std::endl;
    //      delete control_server;
    //      break;
    //    }
    boost::this_thread::sleep(
        boost::posix_time::milliseconds(MAIN_LOOP_SLEEP_TIME_MS));
  }
  shutdown();
  return EXIT_SUCCESS;
}