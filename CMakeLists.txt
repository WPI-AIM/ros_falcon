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
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
rosbuild_add_executable(joystick src/joystick.cpp)
rosbuild_add_executable(gripper src/gripper.cpp)
rosbuild_add_executable(falcon_cs_pid src/falcon_cs_pid.cpp)
rosbuild_add_executable(falcon_js_pid src/falcon_js_pid.cpp)
rosbuild_add_executable(falcon_control src/falcon_control.cpp)
target_link_libraries(joystick nifalcon_cli_base)
target_link_libraries(joystick nifalcon)
target_link_libraries(joystick usb-1.0)
target_link_libraries(falcon_cs_pid nifalcon_cli_base)
target_link_libraries(falcon_cs_pid nifalcon)
target_link_libraries(falcon_cs_pid usb-1.0)
target_link_libraries(falcon_js_pid nifalcon_cli_base)
target_link_libraries(falcon_js_pid nifalcon)
target_link_libraries(falcon_js_pid usb-1.0)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(libnifalcon_cli_base ${PROJECT_NAME})
