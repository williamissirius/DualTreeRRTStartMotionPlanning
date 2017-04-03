<<<<<<< HEAD
# Install script for directory: /media/psf/Dropbox/2017Spring/EECS598/Project/DualTreeRRTStartMotionPlanning/data/wt
=======
# Install script for directory: /home/xz/Desktop/DualTreeRRTStartMotionPlanning/data/wt
>>>>>>> f6de7f4ab0eb25bcd81eba9d907d636dc1d8072f

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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/." TYPE SHARED_LIBRARY FILES "/media/psf/Dropbox/2017Spring/EECS598/Project/DualTreeRRTStartMotionPlanning/data/wt/build/CMakeFiles/CMakeRelink.dir/libmyRRT.so")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/." TYPE SHARED_LIBRARY FILES "/home/xz/Desktop/DualTreeRRTStartMotionPlanning/data/wt/build/CMakeFiles/CMakeRelink.dir/libmyRRT.so")
>>>>>>> f6de7f4ab0eb25bcd81eba9d907d636dc1d8072f
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
<<<<<<< HEAD
file(WRITE "/media/psf/Dropbox/2017Spring/EECS598/Project/DualTreeRRTStartMotionPlanning/data/wt/build/${CMAKE_INSTALL_MANIFEST}"
=======
file(WRITE "/home/xz/Desktop/DualTreeRRTStartMotionPlanning/data/wt/build/${CMAKE_INSTALL_MANIFEST}"
>>>>>>> f6de7f4ab0eb25bcd81eba9d907d636dc1d8072f
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
