//
// Created by sushant on 7/28/19.
//

#ifndef ZMQ_IMAGE_SERVER_TIMER_HPP
#define ZMQ_IMAGE_SERVER_TIMER_HPP
#include <atomic>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
namespace thread_work {
class timer {
public:
  explicit timer(boost::function<void()> job, int time_sleep_ms)
      : boost_timer(io_service_,
                    boost::posix_time::milliseconds(time_sleep_ms)) {
    this->job = job;
    this->time_sleep_ms = time_sleep_ms;
    running = true;
    timer_thread =
        new boost::thread(boost::bind(&timer::initiate_thread_run, this));
  };

  void stop_timer() {
    running = false;
    io_service_.stop();
    timer_thread->join();
  }

  ~timer() {
    if (running) {
      stop_timer();
    }
  };

private:
  std::atomic_bool running;
  boost::asio::io_service io_service_;
  boost::asio::deadline_timer boost_timer;
  boost::function<void()> job;
  boost::thread *timer_thread;
  int time_sleep_ms;
  void initiate_thread_run() {
    boost_timer.async_wait(boost::bind(&timer::run_timed_job, this));
    while (running) {
      io_service_.poll();
      boost::this_thread::sleep(
          boost::posix_time::milliseconds(time_sleep_ms / 2));
    }
  }
  void run_timed_job() {
    if (running) {
      job();
      boost_timer.async_wait(boost::bind(&timer::run_timed_job, this));
    }
  }
};
} // namespace thread_work

#endif // ZMQ_IMAGE_SERVER_TIMER_HPP
