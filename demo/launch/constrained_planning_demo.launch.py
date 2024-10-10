import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Set planning pipeline parameters
    moveit_config.planning_pipelines["ompl"]["panda_arm"][
        "enforce_constrained_state_space"
    ] = True
    moveit_config.planning_pipelines["ompl"]["panda_arm"][
        "projection_evaluator"
    ] = "joints(panda_joint1,panda_joint2)"
    moveit_config.planning_pipelines["ompl"]["panda_arm_hand"][
        "enforce_constrained_state_space"
    ] = True
    moveit_config.planning_pipelines["ompl"]["panda_arm_hand"][
        "projection_evaluator"
    ] = "joints(panda_joint1,panda_joint2)"

    drake_ktopt_planning_pipeline_config = {
        "drake_ktopt": {
            "planning_plugins": [
                "ktopt_interface/KTOptPlanner",
            ],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
        }
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), drake_ktopt_planning_pipeline_config],
    )

    # Demo OMPL constrained planning node
    demo_node = Node(
        package="moveit_drake",
        executable="constrained_planning_demo",
        output="both",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit_drake")
        + "/config/constrained_planning.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription(
        [
            static_tf,
            robot_state_publisher,
            rviz_node,
            run_move_group_node,
            demo_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            hand_controller_spawner,
        ]
    )
