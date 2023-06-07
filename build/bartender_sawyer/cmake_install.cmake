# Install script for directory: /home/loan/sawyer_vision_bartender/src/bartender_sawyer

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/loan/sawyer_vision_bartender/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/loan/sawyer_vision_bartender/build/bartender_sawyer/catkin_generated/installspace/bartender_sawyer.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bartender_sawyer/cmake" TYPE FILE FILES
    "/home/loan/sawyer_vision_bartender/build/bartender_sawyer/catkin_generated/installspace/bartender_sawyerConfig.cmake"
    "/home/loan/sawyer_vision_bartender/build/bartender_sawyer/catkin_generated/installspace/bartender_sawyerConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bartender_sawyer" TYPE FILE FILES "/home/loan/sawyer_vision_bartender/src/bartender_sawyer/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bartender_sawyer" TYPE PROGRAM FILES "/home/loan/sawyer_vision_bartender/build/bartender_sawyer/catkin_generated/installspace/hi.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bartender_sawyer" TYPE PROGRAM FILES "/home/loan/sawyer_vision_bartender/build/bartender_sawyer/catkin_generated/installspace/pour.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bartender_sawyer" TYPE PROGRAM FILES "/home/loan/sawyer_vision_bartender/build/bartender_sawyer/catkin_generated/installspace/get_pos.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bartender_sawyer" TYPE PROGRAM FILES "/home/loan/sawyer_vision_bartender/build/bartender_sawyer/catkin_generated/installspace/gripper_actions.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bartender_sawyer" TYPE PROGRAM FILES "/home/loan/sawyer_vision_bartender/build/bartender_sawyer/catkin_generated/installspace/get_glass.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bartender_sawyer" TYPE PROGRAM FILES "/home/loan/sawyer_vision_bartender/build/bartender_sawyer/catkin_generated/installspace/display.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bartender_sawyer" TYPE PROGRAM FILES "/home/loan/sawyer_vision_bartender/build/bartender_sawyer/catkin_generated/installspace/lights.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bartender_sawyer" TYPE PROGRAM FILES "/home/loan/sawyer_vision_bartender/build/bartender_sawyer/catkin_generated/installspace/camera.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bartender_sawyer" TYPE PROGRAM FILES "/home/loan/sawyer_vision_bartender/build/bartender_sawyer/catkin_generated/installspace/get_glass_with_detection.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bartender_sawyer" TYPE PROGRAM FILES "/home/loan/sawyer_vision_bartender/build/bartender_sawyer/catkin_generated/installspace/initialisation.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bartender_sawyer" TYPE PROGRAM FILES "/home/loan/sawyer_vision_bartender/build/bartender_sawyer/catkin_generated/installspace/take_photo.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bartender_sawyer" TYPE PROGRAM FILES "/home/loan/sawyer_vision_bartender/build/bartender_sawyer/catkin_generated/installspace/pour_with_detection.py")
endif()

