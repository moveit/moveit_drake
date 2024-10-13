ARG ROS_DISTRO=rolling
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

FROM moveit/moveit2:${ROS_DISTRO}-source
LABEL maintainer="Aditya Kamireddypalli kamireddypalliaditya@gmail.com"

ENV ROS_DISTRO=${ROS_DISTRO}
ENV RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
ENV DEBIAN_FRONTEND=noninteractive
ENV WORKSPACE=${ROS_UNDERLAY}/..

COPY . ${WORKSPACE}/src/moveit_drake

RUN apt-get update && \
    apt upgrade && \
    apt install wget -y && \
    wget https://github.com/RobotLocomotion/drake/releases/download/v1.30.0/drake-dev_1.30.0-1_amd64-noble.deb && \
    apt install ./drake-dev_1.30.0-1_amd64-noble.deb -y

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd ${WORKSPACE}/src && \
    vcs import < moveit_drake/.docker/upstream.repos --recursive

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd ${WORKSPACE} && \
    rosdep update --rosdistro=$ROS_DISTRO && \
    apt-get update && \
    apt upgrade -y && \
    rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd ${WORKSPACE} && \
    . install/setup.sh && \
    colcon build --mixin release --parallel-workers 1

# Set up the entrypoint
COPY .docker/ros_entrypoint.sh /sbin/ros_entrypoint.sh
RUN echo "source /sbin/ros_entrypoint.sh" >> ~/.bashrc
ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
