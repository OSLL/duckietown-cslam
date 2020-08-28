FROM duckietown/dt-ros-commons:daffy-amd64
#FROM duckietown/dt-ros-commons:daffy-arm32v7

# Install OpenCV and CvBridge
RUN apt-get update && apt-get install -f -y --allow-unauthenticated \
                      python-opencv ros-kinetic-cv-bridge ros-kinetic-rospy python-pip && apt-get clean
RUN pip install pathos multiprocessing-logging

# You always need to specify:
# ACQ_ROS_MASTER_URI_DEVICE
# ACQ_ROS_MASTER_URI_DEVICE_IP
# ACQ_ROS_MASTER_URI_SERVER
# ACQ_ROS_MASTER_URI_SERVER_IP
# ACQ_DEVICE_NAME # should match the topic, e.g. watchtower10

# Download and install opencv-4.3.0 for the ArUco library
RUN apt-get install -y wget unzip
RUN cd / && wget https://github.com/opencv/opencv/archive/4.3.0.zip && unzip 4.3.0.zip && rm -r 4.3.0.zip
RUN mkdir /opencv-4.3.0/build && cd /opencv-4.3.0/build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local .. && make -j$(nproc) && make install

# Argumens for where the different places from which we will get files from are
ARG aruco_lib_dir=lib/src/aruco
ARG processing_node_dir=ros/src/processing_node
ARG duckietown_msgs_dir=ros/src/duckietown_msgs

# Download the ArUco library and fix the bug in it
COPY ${processing_node_dir}/aruco_bug_fixer.py /${processing_node_dir}/aruco_bug_fixer.py
RUN cd /${processing_node_dir} && wget https://sourceforge.net/projects/aruco/files/3.1.12/aruco-3.1.12.zip && \
    unzip aruco-3.1.12.zip && rm -r aruco-3.1.12.zip && python aruco_bug_fixer.py

# Copy and build the program files and configure the environment variables
COPY ${duckietown_msgs_dir} /${duckietown_msgs_dir}
COPY ${processing_node_dir}/apriltag_processor_node.cpp /${processing_node_dir}/apriltag_processor_node.cpp
COPY ${processing_node_dir}/CMakeLists.txt /${processing_node_dir}/CMakeLists.txt
COPY ${processing_node_dir}/package.xml /${processing_node_dir}/package.xml
RUN /bin/bash -c "cd /ros/src && source /opt/ros/kinetic/setup.bash && catkin_init_workspace && cd .. && catkin_make"
COPY ${processing_node_dir}/config.yml /${processing_node_dir}/config.yml
ENV ACQ_CONFIG_YML /${processing_node_dir}/config.yml

# Set optional environment variables
ENV ACQ_ROS_MASTER_URI_DEVICE_PORT 11311
ENV ACQ_ROS_MASTER_URI_SERVER_PORT 11311
ENV ACQ_TOPIC_RAW camera_node/image/compressed
ENV ACQ_TOPIC_CAMERAINFO camera_node/camera_info
ENV ACQ_TAG_SIZE 0.065
ENV ACQ_POSES_TOPIC poses
ENV ACQ_DETECTOR_TYPE DFC

CMD /bin/bash -c "cd /ros && source /opt/ros/kinetic/setup.bash && source devel/setup.bash && \
                  rosrun processing_node apriltag_processor_node"