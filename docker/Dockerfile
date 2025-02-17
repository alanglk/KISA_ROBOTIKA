FROM ros:humble-ros-base

ARG USER_ID
ARG GROUP_ID

RUN apt update && apt install -y python3-pip stm32flash 

#RUN apt update && apt install -y gazebo-fortress

# Install ROS dependencies
RUN apt update && apt install -y ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \ 
                    ros-humble-ros-gz ros-humble-rqt ros-humble-rqt-graph \ 
                    ros-humble-turtlesim ros-dev-tools \ 
                    ros-humble-ros2-control ros-humble-ros2-controllers \ 
                    ros-humble-kinematics-interface-kdl ros-humble-nav2-common \
                    ros-humble-admittance-controller ros-humble-controller-manager \ 
                    ros-humble-diff-drive-controller ros-humble-joint-state-broadcaster \ 
                    ros-humble-imu-sensor-broadcaster ros-humble-ros-ign \ 
                    ros-humble-ign-ros2-control install ros-humble-ros-ign

RUN groupadd -g ${GROUP_ID} user && \
    useradd -m -u ${USER_ID} -g ${GROUP_ID} user && \
    echo 'user ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/90-user && \
    passwd -d user 
USER user
