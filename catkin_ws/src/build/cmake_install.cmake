# Install script for directory: /home/dat/catkin_ws/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/home/dat/catkin_ws/src/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/home/dat/catkin_ws/src/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.bash;/usr/local/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/dat/catkin_ws/src/build/catkin_generated/installspace/setup.bash"
    "/home/dat/catkin_ws/src/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.sh;/usr/local/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/dat/catkin_ws/src/build/catkin_generated/installspace/setup.sh"
    "/home/dat/catkin_ws/src/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.zsh;/usr/local/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/dat/catkin_ws/src/build/catkin_generated/installspace/setup.zsh"
    "/home/dat/catkin_ws/src/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES "/home/dat/catkin_ws/src/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/dat/catkin_ws/src/build/gtest/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/hector_slam/hector_geotiff_launch/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/hector_slam/hector_slam/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/hector_slam/hector_slam_launch/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/rosserial/rosserial/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/rosserial/rosserial_arduino/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/rosserial/rosserial_mbed/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/rosserial/rosserial_msgs/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/rosserial/rosserial_python/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/rosserial/rosserial_tivac/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/rosserial/rosserial_vex_cortex/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/rosserial/rosserial_vex_v5/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/rosserial/rosserial_xbee/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/rosserial/rosserial_client/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/hector_slam/hector_map_tools/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/hector_slam/hector_nav_msgs/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/turtlebot3/turtlebot3/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/turtlebot3_msgs/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/turtlebot3/turtlebot3_navigation/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/arduino_test_communication/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/hector_slam/hector_geotiff/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/hector_slam/hector_geotiff_plugins/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/hector_slam/hector_marker_drawing/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/mpu9250_ros_driver/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/my_package/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/rosserial/rosserial_server/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/hector_slam/hector_compressed_map_transport/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/jetson_camera/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/rplidar_ros/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/hector_slam/hector_imu_attitude_to_tf/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/hector_slam/hector_imu_tools/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/hector_slam/hector_map_server/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/hector_slam/hector_trajectory_server/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/jetbot_pro/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/localization_data_pub/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/riotu_robot_pose_publisher/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/rosserial/rosserial_embeddedlinux/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/rosserial/rosserial_test/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/rosserial/rosserial_windows/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/hector_slam/hector_mapping/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/turtlebot3/turtlebot3_bringup/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/turtlebot3/turtlebot3_example/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/turtlebot3/turtlebot3_slam/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/turtlebot3/turtlebot3_teleop/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/robot_full20/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/mobile_robot/cmake_install.cmake")
  include("/home/dat/catkin_ws/src/build/turtlebot3/turtlebot3_description/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/dat/catkin_ws/src/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
