FROM apriltag_testing-aruco

RUN apt-get install -y python-tk

# Download and install the Apriltags3 library and configure the environment variables for it
RUN cd / && git clone --recursive https://github.com/duckietown/apriltags3-py.git && cd /apriltags3-py && \
    cp -r apriltags /apriltag_processor_node/apriltag && cp apriltags3.py /apriltag_processor_node/apriltags3.py && \
    mkdir /apriltag_processor_node/apriltag/build && cd /apriltag_processor_node/apriltag/build && \
    cmake .. && make -j$(nproc) install
ENV ACQ_APRILTAG_LIB /apriltag_processor_node/apriltag/
ENV ACQ_APRILTAG_SO /apriltag_processor_node/apriltag/build/lib

# Copy the Python script
COPY apriltag_accuracy_tester.py /apriltag_processor_node

# Set optional environment variables
ENV ACQ_DEVICE_NAME watchtower01
ENV ACQ_TOPIC_RAW camera_node/image/compressed
ENV ACQ_TOPIC_CAMERAINFO camera_node/camera_info
ENV ACQ_DETECTOR_TYPE DFC
ENV ACQ_TAG_SIZE 0.065
ENV INPUT_BAG_PATH /data/rosbag_file.bag

# Start the processes
CMD /bin/bash -c "LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${boost_numpy_dir} && \
                  source /opt/ros/kinetic/setup.bash && \
                  cd /apriltag_processor_node && python apriltag_accuracy_tester.py"