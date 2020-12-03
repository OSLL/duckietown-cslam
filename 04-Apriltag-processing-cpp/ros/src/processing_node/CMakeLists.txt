cmake_minimum_required(VERSION 2.8)
project(processing_node)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        cv_bridge
        std_msgs
        sensor_msgs
        duckietown_msgs
        tf)

include_directories(
        SYSTEM
        ${catkin_INCLUDE_DIRS})

catkin_package()

include_directories(aruco-3.1.12/src)
add_subdirectory(aruco-3.1.12)

include_directories(raspicam-0.1.9/src)
add_subdirectory(raspicam-0.1.9)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(apriltag_processor_node
        apriltag_processor_node.cpp)

target_link_libraries(apriltag_processor_node
        ${catkin_LIBRARIES}
        ${OPENCV_LIBRARIES}
        aruco
        yaml-cpp
        raspicam_cv
        )

target_include_directories(apriltag_processor_node PUBLIC
        ${OPENCV_INCLUDE_DIRS}
        )


include_directories(../../devel/include) # hack for Clion