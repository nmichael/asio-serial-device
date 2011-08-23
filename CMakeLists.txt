CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(asio_serial_device)

SET(CMAKE_BUILD_TYPE Release)

INCLUDE_DIRECTORIES(include)

ADD_LIBRARY(asio_serial_device SHARED src/ASIOSerialDevice.cc)

FIND_PACKAGE(Boost COMPONENTS system thread REQUIRED)
TARGET_LINK_LIBRARIES(asio_serial_device
                                       ${Boost_SYSTEM_LIBRARY}
                                       ${Boost_THREAD_LIBRARY})

SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

INSTALL(TARGETS asio_serial_device DESTINATION lib)
