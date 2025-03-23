# KISA_ROBOTIKA
KISA Robotika repository

## RWander
The implemented strategy is detailed in [this document](./documentation/Robótica_entrega1_AlanGarcía_EnekoPerez_AlexAmenabar.pdf) and a video of the robot can be seen in [this link](https://youtu.be/1kW2iOtpkQQ).

```bash
cd kisa/ros && colcon build --symlink-install
source install/setup.bash
ros2 launch rwander rwander.launch.py
```


## Color follower
Control strategy to follow an especific color object. The implemented results can be seen in
[this video](https://www.youtube.com/watch?v=DOo7WBHmCKk&ab_channel=AlexAmenabar) and more details of the impelementation are explained in [this document](./documentation/Robótica_entrega2_AlanGarcía_EnekoPerez_AlexAmenabar.pdf).

```bash
cd kisa/ros && colcon build --symlink-install
source install/setup.bash
ros2 launch blob_segmentation color_follow.launch.py # modificar launch para utilizar la depth camera
```


## ROS2 useful commands

```bash
printenv |grep -i ROS
ros2 launch gazebo_worlds gz_rosbot.launch.py world:=simple # landmarks
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 launch teleop_twist_joy teleop-launch.py
ros2 run nav2_map_server map_saver_cli -f "simple" --ros-args -p map_subscribe_transient_local:=false
```


## Docker
Build docker ROS2 Humble image for developing:
```bash
docker --debug build --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) -t "kisa-ros:humble" -f "./docker/Dockerfile" .
```


Once the docker container is running use this command to attach a terminal to ir: 
```bash
docker exec -it 57d0325a7216 /bin/bash
```