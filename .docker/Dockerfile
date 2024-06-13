FROM robotlocomotion/drake:1.29.0
MAINTAINER Aditya Kamireddypalli "kamireddypalliaditya@gmail.com"

ENV ROS_DISTRO=humble
ENV WORKSPACE=/root/workspace/src/
ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp

# ROS installation
RUN apt update && \
    apt install locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

RUN apt install -y software-properties-common && \
    add-apt-repository universe

RUN apt update && \
    apt install -y curl git  && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && \
    apt-get upgrade -y && \ 
    apt-get install -y ros-${ROS_DISTRO}-desktop && \
    apt-get install -y python3-rosdep && \
    apt-get install -y python3-vcstool && \
    apt-get install -y python3-colcon-common-extensions wget && \
    apt install -y ros-dev-tools

RUN apt install -y build-essential cmake python3-flake8 python3-setuptools 

RUN pip3 install colcon-mixin && \
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update default

RUN apt update && \
    rosdep init && \
    rosdep update

WORKDIR $WORKSPACE

RUN git clone https://github.com/moveit/moveit2.git -b main && \
    for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; \
    test -r $f && echo $f); do vcs import < "$repo"; done  && \
    apt-get update && \
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    apt remove ros-$ROS_DISTRO-moveit*

WORKDIR $WORKSPACE/..

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp && \
    rosdep update && \
    DEBIAN_FRONTEND=noninteractive \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro \
    ${ROS_DISTRO} && \
    colcon build --mixin release --parallel-workers 1

# create entrypoint
COPY .docker/ros_entrypoint.sh /sbin/ros_entrypoint.sh
RUN sudo chmod 755 /sbin/ros_entrypoint.sh
ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
