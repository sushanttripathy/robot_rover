cmake_minimum_required(VERSION 3.7)
project(robotic_control_client)

set(CMAKE_CXX_STANDARD 11)
ADD_SUBDIRECTORY(proto)

add_executable(robotic_control_client main.cpp robotic_control_client.cpp robotic_control_client.hpp thread_pool.hpp zmq_socket.hpp joystick.cpp joystick.hpp)
SET(Boost_USE_STATIC_LIBS ON)
FIND_PACKAGE(Boost COMPONENTS system thread log REQUIRED)
FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(PC_ZeroMQ QUIET zmq)
FIND_PATH(ZeroMQ_INCLUDE_DIR
        NAMES zmq.hpp
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
        )

FIND_LIBRARY(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
        )

INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${ZeroMQ_INCLUDE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
TARGET_LINK_LIBRARIES(robotic_control_client ${Boost_LIBRARIES} ${OpenCV_LIBS} ${ZeroMQ_LIBRARY} ${PROTOBUF_LIBRARY} proto)