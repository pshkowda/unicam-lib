cmake_minimum_required(VERSION 3.15)
project(libunicam)

set(CMAKE_CXX_STANDARD 14)

SET(MY_DIR ${libunicam_SOURCE_DIR})
FIND_PACKAGE(PkgConfig)    # try find PKGConfig as it will be used if found
FIND_PACKAGE(realsense2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 4 REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${realsense_INCLUDE_DIR}
)

set(LINK_LIBS  ${OpenCV_LIBRARIES} ${realsense2_LIBRARY})
link_directories(${LINK_LIBS})

add_executable(libunicam
        ./headers/async_buf.h
        headers/RealsenseProvider.h
        headers/RealsenseDevice.h
        main.cpp headers/unicam/UnicamApi.h
        headers/unicam/UnicamCamera.h
        headers/unicam/UnicamDeviceProvider.h
        headers/unicam/UnicamDevices.h
        src/RealsenseProvider.cpp
        src/RealsenseDevice.cpp
        headers/RealsenseCamera.h
		src/CameraFrameSaver.cpp
		headers/frameSaver/CameraFrameSaver.h
         headers/frame_data.h
		headers/orientationControl/CameraOrientationController.h)

target_link_libraries(libunicam ${LINK_LIBS})