## Apriltag processor (notes)

Предполагается использование прошивки RPI версии `master19`.

На RPI может плохо работать `avahi-daemon`, из-за чего при включении RPI не будет доступна по имени.
Для запуска `avahi-daemon` нужно зайти по ssh на RPI: `ssh duckie@<rpi ip>` и выполнить команду
`sudo service avahi-daemon start`.

Как задать нужное разрешение камеры и записать данные калибровки, написано в `README.md` ветки
`new_marker_detector_only_cpp`.

-------------------

Пример запуска контейнера на PRI:
```
docker -H <rpi name>.local run -it --rm --network=host -e ROS_MASTER_URI=http://<rpi ip>:11311 \
       -e ROS_HOSTNAME=<rpi ip> -e ACQ_DEVICE_NAME=<rpi name> --name apriltag_processor <docker image name>
```
