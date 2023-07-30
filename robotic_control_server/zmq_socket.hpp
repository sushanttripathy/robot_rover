//
// Created by sushant on 7/28/19.
//

#ifndef ZMQ_IMAGE_SERVER_ZMQ_SOCKET_HPP
#define ZMQ_IMAGE_SERVER_ZMQ_SOCKET_HPP
#include <string>
#include <zmq.hpp>
namespace zmq_sockets {
class zmq_server_socket {
public:
  zmq_server_socket(std::string host, int port, int num_io_threads)
      : context(num_io_threads), socket(context, ZMQ_REP) {
    std::stringstream stringstream1;
    stringstream1 << "tcp://" << host << ":" << port;
    socket.bind(stringstream1.str());
  }
  ~zmq_server_socket() { socket.close(); }

  zmq::detail::send_result_t send(zmq::const_buffer &data,
                                  zmq::send_flags flags) {
    return socket.send(data, flags);
  }

  zmq::detail::recv_result_t recv(zmq::message_t &request,
                                  zmq::recv_flags flags) {
    return socket.recv(request, flags);
  }

private:
  zmq::context_t context;
  zmq::socket_t socket;
};

class zmq_client_socket {
public:
  zmq_client_socket(std::string host, int port, int num_io_threads)
      : context(num_io_threads), socket(context, ZMQ_REQ) {
    std::stringstream stringstream1;
    stringstream1 << "tcp://" << host << ":" << port;
    socket.connect(stringstream1.str());
  }
  ~zmq_client_socket() { socket.close(); }

  zmq::detail::send_result_t send(zmq::const_buffer &data,
                                  zmq::send_flags flags) {
    return socket.send(data, flags);
  }

  zmq::detail::recv_result_t recv(zmq::message_t &request,
                                  zmq::recv_flags flags) {
    return socket.recv(request, flags);
  }

private:
  zmq::context_t context;
  zmq::socket_t socket;
};

} // namespace zmq_sockets
#endif // ZMQ_IMAGE_SERVER_ZMQ_SOCKET_HPP