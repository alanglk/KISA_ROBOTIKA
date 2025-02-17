# KISA_ROBOTIKA
KISA Robotika repository

Build docker ROS2 Humble image for developing:
```bash
docker --debug build --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) -t "kisa-ros:humble" -f "./docker/Dockerfile" .
```


Once the docker container is running use this command to attach a terminal to ir:
```bash
docker exec -it 57d0325a7216 /bin/bash
```


## ROS2 useful commands

```bash
printenv |grep -i ROS
ros2 launch gazebo_worlds gz_rosbot.launch.py world:=simple
```