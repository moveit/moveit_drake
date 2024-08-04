# Experimental MoveIt 2 - Drake Integration

Under construction

## Docker Workflow (Preferred and tested)

### Requirements
`docker` and `docker-compose` - Follow instructions [here](https://docs.docker.com/engine/install/ubuntu/).

### Steps
The following steps clone and build the base image that you will require to
test/build/run/develop with the repo (and will take some time, as it builds
moveit)

    git clone https://github.com/moveit/moveit_drake.git
    cd moveit_drake
    docker compose build

This should give you an image with `drake` and `moveit2`.
Next, create a container with the following and create shell access.

    docker compose up
    docker compose exec -it moveit_drake bash

Follow [instructions](#build-moveit_drake) below to build `moveit_drake`


## Local Installation

### Install Drake

[Follow these instructions](https://drake.mit.edu/installation.html)

### Build `moveit_drake`

Follow the [MoveIt Source Build](https://moveit.ros.org/install-moveit2/source/) instructions to set up a colcon workspace with MoveIt from source.

Open a command line to your colcon workspace:

    cd ${WORKSPACE}/src

Download the MoveIt Tutorials source code:

    git clone https://github.com/moveit/moveit_drake.git
    vcs import < moveit_drake/moveit_drake.repos
    rosdep install -r --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -y

Configure and build the workspace:

    cd ${WORKSPACE}
    colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release

### Run the demo

```
ros2 launch moveit_drake pipeline_testbench.launch.py
```

### Development

- Use [pre-commit to format your code](https://moveit.ros.org/documentation/contributing/code/#pre-commit-formatting-checks)

# Todo section

This section keeps a list of immediate todos, will be deleted before repo release

- [x] Create drake planning pipeline option in `pipeline_testbench.launch.py`
- [x] Declare to moveit, to use the drake ktopt planning pipeline
- [ ] Build planner manager and planning context to display info from `moveit`
  and `drake` instance.
    - [ ] Generated placeholder classes mimicking `stomp` implementation.
    - [ ] Display info messages during testbench runtime.
    - [ ]
- [ ] read Robot description and display onto drake visualizer

### Doubts
- [x] stomp_moveit::ParamListener, where is this being declared
- [ ] Why is the parameter file in the "res" directory

### Potential issues
- Assumes that planner managers initialize will set robot description before a
  call to getPlanningContext.

### Some helper commands
To just rebuild `moveit_drake`
```
rm -rf build/moveit_drake install/moveit_drake
colcon build --packages-select moveit_drake
```
