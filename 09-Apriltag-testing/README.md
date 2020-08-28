## Apriltag accuracy tester

Программа считывает по 1 кадру из rosbag-файла (используются топики 
`/watchtower01/camera_node/image/compressed` и `/watchtower01/camera_node/camera_info`), 
запускает детекцию в библиотеках Apriltags3 и ArUco, затем строит график полученных траекторий.

Вручную запускать докерфайл не нужно, его использует `launcher.bash`.
Пример запуска:
```
./launcher.bash <path to bag-file dir> <bag-file name>
```

Если маркер на кадре был распознан обеими библиотеками, то его координаты из ArUco рисуются красным
кругом, из Apriltags3 — синим, если только библиотекой ArUco — фиолетовым, только Apriltags3 — голубым.

-------------------
* Может присутствовать проблема с масштабом z-оси. Для получения корректных глубин на графике
можно изменить коэффициент `1.25` в строчке
```
trs[i][marker] = map(lambda t: [t[0], t[1], t[2] / 1.25], trs[i][marker])  # odd bug with z-scale
```