FROM duckietown/dt-ros-commons:daffy-amd64

# Argumens for where the different places from which we will get files from are
ARG tester_node_dir=ros/src/tester_node
ARG duckietown_msgs_dir=ros/src/duckietown_msgs

# Copy the message files
COPY ${duckietown_msgs_dir} /${duckietown_msgs_dir}
RUN /bin/bash -c "cd /ros/src && source /opt/ros/noetic/setup.bash && catkin_init_workspace && cd .. && catkin_make"

# Set optional environment variables
ENV ACQ_ROS_MASTER_URI_DEVICE_PORT 11311
ENV ACQ_ROS_MASTER_URI_SERVER_PORT 11311
ENV ACQ_TOPIC_RAW camera_node/image/compressed
ENV ACQ_TOPIC_CAMERAINFO camera_node/camera_info
ENV ACQ_TAG_SIZE 0.065
ENV ACQ_POSES_TOPIC poses
ENV ACQ_DETECTOR_TYPE DFC
ENV ACQ_IMAGE_PROCESSORS 4
ENV INPUT_BAG_PATH /data/rosbag_file.bag

ARG BAG_PATH
#ARG INPUT_BAG_PATH=${INPUT_BAG_PATH}
COPY ${BAG_PATH} /data/rosbag_file.bag

# Copy and build the program files and configure the environment variables
COPY ${tester_node_dir} /${tester_node_dir}
RUN chmod +x /${tester_node_dir}/wrapper.sh

#CMD /bin/bash -c "cd /ros && source /opt/ros/noetic/setup.bash && source /ros/devel/setup.bash && \
#                  roslaunch tester_node tester.launch"
#                  cd /ros/src/tester_node && ./wrapper.sh"
#CMD /bin/bash -c "cd /ros/src/tester_node && ./wrapper.sh"
CMD /bin/bash -c "cd /ros && source /opt/ros/noetic/setup.bash && source /ros/devel/setup.bash && \
                  cd /ros/src/tester_node && ./wrapper.sh"