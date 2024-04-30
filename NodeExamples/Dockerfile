FROM osrf/ros:humble-desktop

# Argument to allow network domain id to be set at build time
ARG NET_ID

# Environment
ENV ROS_DISTRO humble
ENV DEV_WS /app/practical_ws
ENV GAZEBO /usr/share/gazebo
ENV HOME /app
ENV DISPLAY host.docker.internal:0.0
ENV LIB_GL_ALWAYS_INDIRECT 0
ENV ROS_DOMAIN_ID ${NET_ID:-0}

# Install tools
RUN apt-get update && apt-get install -y \
      git \
      ros-$ROS_DISTRO-rqt-tf-tree \
      ros-$ROS_DISTRO-xacro \
      python3-pip \
      python3-colcon-common-extensions \
      python-is-python3\
      curl \
      nautilus \
      nano && \
      rm -rf /var/lib/apt/lists/*

# Python items
RUN /bin/bash -c '. /opt/ros/${ROS_DISTRO}/setup.bash; \
        pip3 install transforms3d; \
        pip3 install pyclean; \
        pip3 install numpy; \
        pip3 install matplotlib'

# Install ROS package dependencies
RUN apt-get update && \
    rosdep update 

# Navigation2 and Gazebo
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-gazebo-* \
    ros-$ROS_DISTRO-ros-gz \
    # For the house:
    ros-$ROS_DISTRO-turtlebot3-simulations

# Development code
RUN mkdir -p $DEV_WS/src
WORKDIR $DEV_WS/src
COPY ./practical_ws/src . 

# Build and install
WORKDIR $DEV_WS
RUN /bin/bash -c '. /opt/ros/${ROS_DISTRO}/setup.bash; \
        colcon build --symlink-install --parallel-workers 1'

# Set up ROS source

RUN sed --in-place --expression \
      '$isource /opt/ros/'$ROS_DISTRO'/setup.bash' \
      /ros_entrypoint.sh
RUN sed --in-place --expression \
      '$isource '$DEV_WS'/install/setup.bash' \
      /ros_entrypoint.sh
RUN sed --in-place --expression \
      '$isource '$GAZEBO'/setup.sh' \
      /ros_entrypoint.sh
# Enable colcon_cd
RUN sed --in-place --expression \
      '$isource /usr/share/colcon_cd/function/colcon_cd.sh' \
      /ros_entrypoint.sh
RUN sed --in-place --expression \
      '$iexport _colcon_cd_root='$DEV_WS'' \
      /ros_entrypoint.sh
# Enable autocompletion colcon
RUN sed --in-place --expression \
      '$isource /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' \
      /ros_entrypoint.sh

RUN /bin/bash -c '. /opt/ros/${ROS_DISTRO}/setup.bash; source /ros_entrypoint.sh'

