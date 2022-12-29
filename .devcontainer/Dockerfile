###########################################
# Base image 
###########################################
FROM ros:humble-ros-base

# install Jetson GPIO c++ library (port of Python GPIO)
RUN cd $HOME \
  && git clone https://github.com/pjueon/JetsonGPIO.git \
  && cd JetsonGPIO \
  && mkdir -p build \
  && cd build \
  && cmake -DBUILD_EXAMPLES= ON -DJETSON_GPIO_POST_INSTALL=ON ../ \
  && make -j`nproc` \
  && make install \
  && cd $HOME \
  $$ rm -rf JetsonGPIO

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [Optional] Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo python3-pip libi2c-dev \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && sudo usermod -a -G root,adm,cdrom,sudo,audio,dip,video,plugdev,gpio,i2c $USERNAME \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

USER $USERNAME

ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1
# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1

ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
