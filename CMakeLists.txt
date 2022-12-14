cmake_minimum_required(VERSION 2.8.3)
project(dreamvu_stereo)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  message_generation
  message_runtime
  
)


catkin_package(
   LIBRARIES dreamvu_pal_camera 
   CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs message_generation nav_msgs geometry_msgs
  message_runtime
)
set(PAL_INCLUDE_DIR $ENV{HOME}/DreamVu/PAL/include )
set(PAL_LIBRARY /usr/local/lib/libPAL.so )
include_directories(
        ${PAL_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS} 
        ${OpenCV_INCLUDE_DIRS} 

)

set(AVAILABLE_ROS_VERSIONS "melodic;lunar;kinetic;jade;indigo;noetic")

foreach(version ${AVAILABLE_ROS_VERSIONS})
	find_path(ROS_LIBS libroscpp.so PATHS /opt/ros/${version}/lib)
endforeach()


link_directories(
        ${ROS_LIBS}
        /usr/include/libusb-1.0
)

set(CMAKE_CXX_FLAGS "-w")

set(CATKIN_LIBRARIES libudev.so cv_bridge.so image_transport.so message_filters.so tinyxml2.so class_loader.so dl.so roslib.so rospack.so python2.7.so boost_program_options.so tinyxml.so roscpp.so boost_filesystem.so boost_signals.so rosconsole.so rosconsole_log4cxx.so rosconsole_backend_interface.so log4cxx.so boost_regex.so xmlrpcpp.so roscpp_serialization.so rostime.so cpp_common.so boost_system.so boost_thread.so boost_chrono.so boost_date_time.so boost_atomic.so pthread.so console_bridge.so laser_scan_filters.so)

find_package(PkgConfig REQUIRED)
pkg_search_module(PKG_OPENCV REQUIRED opencv)

add_executable(stereo src/stereo_node.cpp )
add_dependencies(stereo ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(stereo ${CATKIN_LIBRARIES} ${PKG_OPENCV_LDFLAGS} ${V4l2_LIBRARIES} ${PAL_LIBRARY})
