FROM ros:humble-ros-base

ARG USER_ID
ARG GROUP_ID

RUN apt update && apt install -y ros-humble-ros-gz

RUN groupadd -g ${GROUP_ID} user && \
    useradd -m -u ${USER_ID} -g ${GROUP_ID} user && \
    echo 'user ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/90-user && \
    passwd -d user 
USER user
