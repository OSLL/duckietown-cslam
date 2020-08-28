#!/bin/bash

echo "Path to bag-file folder: $1"
echo "Bag-file: $2"
if [ "$1" = "" ] || [ "$2" = "" ]
then
    echo "Must specify path to bag-file folder and its name"
    exit 1
fi

docker build -t apriltag_testing-aruco -f ../04-Apriltag-processing/Dockerfile ../04-Apriltag-processing
docker build -t apriltag_testing .
xhost +
docker run -it --rm -e DISPLAY=unix$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $1:/data \
       -e INPUT_BAG_PATH=/data/$2 apriltag_testing
xhost -
