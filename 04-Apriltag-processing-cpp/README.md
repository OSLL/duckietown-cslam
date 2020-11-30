## Only cpp apriltag processor

* ArUco detection library 
* Retrieving camera images in the same container (no unnecessary copying of images)
* All code in c++

*TODO: add debug capability (publish images, calibration data, etc.)*

Camera calibration data is in `calibr.yaml` file

To provide access to the RPI camera it is required to stop `duckiebot-interface` container on RPI:
```shell script
docker -H <rpi name>.local stop duckiebot-interface
```

However, it will be useful to stop all other RPI containers for speedup

-------------------
An example of building a docker image on PRI (be prepared that `OpenCV` will take a very long time to compile;
you can also use a pre-built image -- see `Dockerfile`):
```shell script
docker -H <rpi name>.local build -t <docker image name> .
```
An example of starting a container on PRI:
```shell script
docker -H <rpi name>.local run -it --rm --network=host --privileged \
       -e ACQ_DEVICE_NAME=<rpi name> --name apriltag_processor <docker image name>
```

-------------------
To visualize trajectories of markers:
* Start `roscore` on your computer
```shell script
roscore
```
* Start graph optimizer
```shell script
docker run --rm -it --net=host -e OUTPUT_DIR=/data --name graph_optimizer \
       -v <path to output folder>:/data duckietown/cslam-graphoptimizer:daffy-amd64
```
* Start visualization and add `movable_path` topics in opened `rviz` to draw
```shell script
xhost + ; docker run -it --rm --net=host --env="DISPLAY" \
          -e ROS_MASTER_IP=<computer roscore ip> duckietown/cslam-visualization ; xhost -
```
* Start detection on PRI
```shell script
docker -H <rpi name>.local run -it --rm --network=host --privileged \
       -e ROS_MASTER_URI=http://<computer roscore ip>:11311 \
       -e ACQ_DEVICE_NAME=<rpi name> --name apriltag_processor <docker image name>
```
Visualization in `rviz` may lag behind the actual movements of the markers over time