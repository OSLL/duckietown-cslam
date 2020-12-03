FIND_PACKAGE(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
SET(LIB_COMPONENTS ${LIB_COMPONENTS} ${PROJECT_NAME}_cv)
SET(PROJECT_CV_CREATED_FLAG "YES")

find_package(catkin REQUIRED COMPONENTS
        rosbag
        cv_bridge)

#find_package(catkin REQUIRED COMPONENTS
#        rosbag
#        rosconsole
#        roscpp
#        )



#include_directories(/opt/ros/noetic/include/)
#link_directories(/opt/ros/noetic/lib/)


#MESSAGE("aaaaaa" ${rosbag_INCLUDE_DIRS})
#include_directories(
#        ${rosbag_INCLUDE_DIRS})

ADD_LIBRARY(raspicam raspicam_cv.h raspicam_cv.cpp)
ADD_LIBRARY(raspicam_cv raspicam_cv.h raspicam_cv.cpp)

#find_library(rosbag REQUIRED)
TARGET_LINK_LIBRARIES(raspicam ${rosbag_LIBRARIES})
TARGET_LINK_LIBRARIES(raspicam_cv ${rosbag_LIBRARIES})
TARGET_LINK_LIBRARIES(raspicam ${catkin_LIBRARIES})
TARGET_LINK_LIBRARIES(raspicam_cv ${catkin_LIBRARIES})



#INSTALL(TARGETS raspicam
#RUNTIME DESTINATION bin COMPONENT main
#LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
#ARCHIVE DESTINATION lib COMPONENT main)
#INSTALL(TARGETS raspicam_cv
#RUNTIME DESTINATION bin COMPONENT main
#LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
#ARCHIVE DESTINATION lib COMPONENT main)
#
#INSTALL(FILES cvversioning.h raspicam_cv.h
#        DESTINATION include/raspicam
#        COMPONENT main)

