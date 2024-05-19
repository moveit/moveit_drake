# Experimental MoveIt 2 - Drake Integration

Under construction

## Installation

### Install Drake

[Follow these instructions](https://drake.mit.edu/installation.html)

### Build moveit_drake

Follow the [MoveIt Source Build](https://moveit.ros.org/install-moveit2/source/) instructions to set up a colcon workspace with MoveIt from the source.

Open a command line to your colcon workspace:

    cd YOUR_WORKSPACE/src

Download the MoveIt Tutorials source code:

    git clone https://github.com/moveit/moveit_drake.git
    vcs import < moveit_drake/moveit_drake.repos
    rosdep install -r --from-paths . --ignore-src --rosdistro rolling -y

Configure and build the workspace:

    cd $COLCON_WS
    nice colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release

### Run the demo

```
ros2 launch moveit_drake pipeline_testbench.launch.py
```

### Development

- Use [pre-commit to format your code](https://moveit.ros.org/documentation/contributing/code/#pre-commit-formatting-checks)