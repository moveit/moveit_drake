FROM robotlocomotion/drake:1.29.0
MAINTAINER Aditya Kamireddypalli "kamireddypalliaditya@gmail.com"

ENV ROS_DISTRO=humble
ENV WORKSPACE=/root/workspace/
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV DEBIAN_FRONTEND=noninteractive

# ROS installation
RUN apt update && \
    apt install locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

RUN apt install -y software-properties-common && \
    add-apt-repository universe

RUN apt update && \
    apt install -y curl git && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && \
    apt upgrade -y && \
    apt install -y \
        build-essential \
        cmake \
        git-lfs \
        python3-flake8 \
        python3-rosdep \
        python3-setuptools \
        python3-vcstool \
        python3-colcon-common-extensions \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-dev-tools \
        wget

RUN pip3 install colcon-mixin && \
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update default

RUN apt update && \
    rosdep init && \
    rosdep update

# Setup the workspace
WORKDIR ${WORKSPACE}
RUN mkdir -p src

# Install dependencies of this package
# NOTE: Copying only the necessary files to not bust cache as much.
COPY moveit_drake.repos ${WORKSPACE}/src/moveit_drake/
WORKDIR ${WORKSPACE}/src/
RUN vcs import < moveit_drake/moveit_drake.repos

WORKDIR ${WORKSPACE}
COPY package.xml ${WORKSPACE}/src/moveit_drake/
RUN apt update && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y && \
    rm -rf src/moveit_drake

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --mixin release --parallel-workers 1

# Set up the entrypoint
COPY .docker/ros_entrypoint.sh /sbin/ros_entrypoint.sh
RUN echo "source /sbin/ros_entrypoint.sh" >> ~/.bashrc
ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
