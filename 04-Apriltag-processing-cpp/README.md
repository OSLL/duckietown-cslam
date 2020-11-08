## Only cpp apriltag processor

It assumes the use of RPI firmware version `master19`.

To speed up the work of the detector, images from the RPI camera are received uncompressed. This requires replacing 2 scripts in the `dt18_03_roscore_duckiebot-interface_1` container: `cam_info_reader_node.py` and `camera_node_sequence.py`.
This can be done with the following command (example for `cam_info_reader_node.py`):
```
scp <path to cam_info_reader_node.py> <rpi name>:/home/duckie/cam_info_reader_node.py && \
ssh <rpi name> docker cp /home/duckie/cam_info_reader_node.py dt18_03_roscore_duckiebot-interface_1:/home/duckiebot-interface/catkin_ws/src/camera_driver/src/cam_info_reader_node.py && \
docker -H <rpi name>.local restart dt18_03_roscore_duckiebot-interface_1
```

You also need to change the resolution of the images to 1296x976 (!not x972) in the file `/home/duckiebot-interface/catkin_ws/src/duckietown/config/baseline/camera_driver/camera_node/default.yaml`.
And write the calibration data to `/data/config/calibrations/camera_intrinsic/default.yaml`.
Sample files are also in the `dt-duckiebot-interface folder`.

-------------------
To build a docker image on an RPI, you need to change the base image in the dockerfile from `daffy-amd64` to `daffy-arm32v7`.
An example of starting a container on PRI:
```
docker -H <rpi name>.local run -it --rm --network=host -e ROS_MASTER_URI=http://<rpi ip>:11311 \
       -e ROS_HOSTNAME=<rpi ip> -e ACQ_DEVICE_NAME=<rpi name> --name apriltag_processor <docker image name>
```
