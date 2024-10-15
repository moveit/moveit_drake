ARG ROS_DISTRO=rolling
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ARG DRAKE_VERSION=1.30.0-1

FROM moveit/moveit2:${ROS_DISTRO}-source
LABEL maintainer="Aditya Kamireddypalli kamireddypalliaditya@gmail.com"

ARG ROS_DISTRO
ARG RMW_IMPLEMENTATION
ARG DRAKE_VERSION

ENV DEBIAN_FRONTEND=noninteractive
ENV WORKSPACE=${ROS_UNDERLAY}/..

COPY . ${WORKSPACE}/src/moveit_drake

RUN apt-get update && \
    apt-get install --no-install-recommends \
        ca-certificates \
        gnupg \
        lsb-release \
        python3-pip \
        wget -y && \
    pip3 install --break-system-packages pre-commit && \
    wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - \
        | sudo tee /etc/apt/trusted.gpg.d/drake.gpg >/dev/null && \
    echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/drake.list >/dev/null && \
    apt-get update && \
    apt-get install --no-install-recommends drake-dev=${DRAKE_VERSION} -y

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd ${WORKSPACE}/src && \
    vcs import < moveit_drake/.docker/upstream.repos --recursive

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd ${WORKSPACE} && \
    rosdep update --rosdistro=$ROS_DISTRO && \
    apt-get update && \
    apt upgrade -y && \
    rosdep install --from-paths src --ignore-src -r -y && \
    apt-get install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp -y

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd ${WORKSPACE} && \
    . install/setup.sh && \
    colcon build --mixin release --parallel-workers 1

# Set up the entrypoint
COPY .docker/ros_entrypoint.sh /sbin/ros_entrypoint.sh
RUN echo "source /sbin/ros_entrypoint.sh" >> ~/.bashrc
ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
