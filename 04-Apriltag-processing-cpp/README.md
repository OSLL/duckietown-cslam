## Only cpp apriltag processor

Предполагается использование прошивки RPI версии `master19`.

Для ускорения работы детектора изображения с RPI-камеры принимаются в несжатом виде. Это требует замены 2
скриптов в контейнере `dt18_03_roscore_duckiebot-interface_1`: `cam_info_reader_node.py` и `camera_node_sequence.py`.
Сделать это можно следующей командой (пример для `cam_info_reader_node.py`):
```
scp <path to cam_info_reader_node.py> <rpi name>:/home/duckie/cam_info_reader_node.py && \
ssh <rpi name> docker cp /home/duckie/cam_info_reader_node.py dt18_03_roscore_duckiebot-interface_1:/home/duckiebot-interface/catkin_ws/src/camera_driver/src/cam_info_reader_node.py && \
docker -H <rpi name>.local restart dt18_03_roscore_duckiebot-interface_1
```

Также нужно изменить разрешение изображений на 1296x976 (!не x972) в файле 
`/home/duckiebot-interface/catkin_ws/src/duckietown/config/baseline/camera_driver/camera_node/default.yaml`. 
И записать данные калибровки в `/data/config/calibrations/camera_intrinsic/default.yaml`. 
Примеры файлов также в папке `dt-duckiebot-interface`.

-------------------
Для сборки докер-образа на RPI нужно поменять базовый образ в докерфайле с `daffy-amd64` на `daffy-arm32v7`.
Пример запуска контейнера на PRI:
```
docker -H <rpi name>.local run -it --rm --network=host -e ROS_MASTER_URI=http://<rpi ip>:11311 \
       -e ROS_HOSTNAME=<rpi ip> -e ACQ_DEVICE_NAME=<rpi name> --name apriltag_processor <docker image name>
```
