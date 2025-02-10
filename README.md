# KISA_ROBOTIKA
KISA Robotika repository

Build docker ROS2 Humble image for developing:
```bash
docker --debug build --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) -t "kisa-ros:humble" -f "./docker/Dockerfile" .
```

