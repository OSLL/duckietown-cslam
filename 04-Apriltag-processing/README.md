## Apriltag processor (notes)

It assumes the use of RPI firmware version `master19`.

On RPI, `avahi-daemon` may not work well, due to which, once enabled, the RPI will not be available by name.
To start `avahi-daemon`, you need to ssh to the RPI: `ssh duckie@<rpi ip>` and execute the command `sudo service avahi-daemon start`.

How to set the required camera resolution and record the calibration data is written in the `README.md` in the `new_marker_detector_only_cpp` branch.

-------------------

An example of starting a container on PRI:
```
docker -H <rpi name>.local run -it --rm --network=host -e ROS_MASTER_URI=http://<rpi ip>:11311 \
       -e ROS_HOSTNAME=<rpi ip> -e ACQ_DEVICE_NAME=<rpi name> --name apriltag_processor <docker image name>
```
