## Only cpp apriltag processor

* ArUco detection library 
* Retrieving camera images in the same container
* All code in c++ (no unnecessary copying of images, etc.)

*TODO: add debug capability (publish images, calibration data, etc.)*

Camera calibration data is in `calibr.yaml` file

-------------------
An example of building a docker image on PRI:
```
docker -H <rpi name>.local build -t <docker image name> .
```
An example of starting a container on PRI:
```
docker -H <rpi name>.local run -it --rm --network=host --privileged -e ROS_MASTER_URI=http://<rpi ip>:11311 \
       -e ROS_HOSTNAME=<rpi ip> -e ACQ_DEVICE_NAME=<rpi name> --name apriltag_processor <docker image name>
```
