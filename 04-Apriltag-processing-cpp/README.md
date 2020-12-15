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
Or you can use ready image `galanton/apriltag-processor:daffy-arm32v7`

An example of starting a container on PRI:
```shell script
docker -H <rpi name>.local run -it --rm --network=host --privileged \
       -e ACQ_DEVICE_NAME=<rpi name> --name apriltag_processor <docker image name>
```

-------------------
To visualize the trajectories please use the version before migration to AprilTagDetectionArray:
https://github.com/OSLL/duckietown-cslam/tree/173a193b3429ee9fe87be1cc6ff4d179c18422bb/04-Apriltag-processing-cpp
