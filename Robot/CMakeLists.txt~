cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
rosbuild_find_ros_package(rosserial_arduino)
set(ARDUINO_SDK_PATH /usr/share/arduino)
set(ARDUINO_SDK_VERSION 1.0)

include(${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/rosserial.cmake)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
qt4_wrap_cpp(moc_sources ${headers})
include_directories(${CMAKE_BINARY_DIR})
include_directories(${QT_INCLUDE_DIR})

set(FIRMWARE_NAME chatter)

set(${FIRMWARE_NAME}_BOARD atmega328)         # Arduino Target board
set(${FIRMWARE_NAME}_SRCS src/chatter.cpp)
set(${FIRMWARE_NAME}_PORT /dev/ttyUSB0)            # Serial upload port
generate_ros_firmware(${FIRMWARE_NAME})
