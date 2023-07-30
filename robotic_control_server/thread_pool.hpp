//
// Created by sushant on 7/28/19.
//

#ifndef ZMQ_IMAGE_SERVER_THREAD_POOL_HPP
#define ZMQ_IMAGE_SERVER_THREAD_POOL_HPP

#include <atomic>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
namespace thread_work {
class thread_pool {
public:
  explicit thread_pool(size_t size) : work_(io_service_) {
    for (size_t i = 0; i < size; ++i) {
      workers_.create_thread(
          boost::bind(&boost::asio::io_service::run, &io_service_));
    }
    running = true;
  }

  void join_all() {
    io_service_.stop(); // stop before join_all
    workers_.join_all();
    running = false;
  }

  ~thread_pool() {
    if (running) {
      join_all();
    }
  }

  template <class F> void add_work(F f) { io_service_.post(f); }

private:
  boost::thread_group workers_;
  boost::asio::io_service io_service_;
  boost::asio::io_service::work work_;
  std::atomic_bool running;
};
};     // namespace thread_work
#endif // ZMQ_IMAGE_SERVER_THREAD_POOL_HPP
