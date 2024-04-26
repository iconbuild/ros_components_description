# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.conditions import  LaunchConfigurationNotEquals
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


# The frame of the point cloud from ignition gazebo 6 isn't provided by <frame_id>.
# See https://github.com/gazebosim/gz-sensors/issues/239
def fix_depth_image_tf(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    device_namespace = LaunchConfiguration("device_namespace").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    camera_name = LaunchConfiguration("camera_name").perform(context)

    device_namespace_ext = device_namespace + "/"
    if device_namespace == "":
        device_namespace_ext = ""

    tf_prefix_ext = tf_prefix + "_"
    if tf_prefix == "":
        tf_prefix_ext = ""

    parent_frame = tf_prefix_ext + camera_name + "_depth_optical_frame"
    child_frame = (
        "panther/base_link//"
        + device_namespace_ext
        + tf_prefix_ext
        + camera_name
        + "_orbbec_astra_depth"
    )

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="point_cloud_tf",
        output="log",
        arguments=["0", "0", "0", "1.57", "-1.57", "0", parent_frame, child_frame],
        parameters=[{"use_sim_time": True}],
        namespace=robot_namespace,
    )
    return [static_transform_publisher]

def generate_launch_description():
    robot_namespace = LaunchConfiguration("robot_namespace")
    device_namespace = LaunchConfiguration("device_namespace")
    tf_prefix = LaunchConfiguration("tf_prefix")
    controllers_file = LaunchConfiguration("controllers_file")

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("ros_components_description"), "config", controllers_file]
    )

    namespace_warn = LogInfo(
        msg="Namespace is not implemented with manipulators. Look here: https://github.com/ros-controls/ros2_control/issues/1506",
        condition=LaunchConfigurationNotEquals(robot_namespace, "None"),
    )

    gz_bridge_name = LaunchConfiguration("gz_bridge_name")
    gz_bridge_config_path = PathJoinSubstitution(
        [FindPackageShare("ros_components_description"), "config", "gz_kinova_remappings.yaml"]
    )

    namespaced_gz_bridge_config_path = ReplaceString(
        source_file=gz_bridge_config_path,
        replacements={
            "<robot_namespace>": robot_namespace,
            "<device_namespace>": device_namespace,
        },
    )

    # Using tf as namespace is caused by
    # https://github.com/ros-controls/ros2_control/issues/1506
    # After this fix the device_namespace should be used.
    namespaced_initial_joint_controllers_path = ReplaceString(
        source_file=initial_joint_controllers,
        replacements={
            "- joint": ["- ", tf_prefix, "joint"],
            "  joint_trajectory_controller:": ["  ", tf_prefix, "joint_trajectory_controller:"],
            "  robotiq_gripper_controller:": ["  ", tf_prefix, "robotiq_gripper_controller:"],
            "robotiq_85_left_knuckle_joint": [tf_prefix, "robotiq_85_left_knuckle_joint"],
        },
    )

    declare_device_namespace = DeclareLaunchArgument(
        "device_namespace",
        default_value="",
        description="Sensor namespace that will appear before all non absolute topics and TF frames, used for distinguishing multiple cameras on the same robot.",
    )

    declare_robot_namespace = DeclareLaunchArgument(
        "robot_namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Namespace which will appear in front of all topics (including /tf and /tf_static).",
    )

    declare_tf_prefix = DeclareLaunchArgument(
        "tf_prefix",
        default_value="",
        description="Prefix added for all links of device. Here used as fix for static transform publisher.",
    )

    declare_start_joint_controller = DeclareLaunchArgument(
        "start_joint_controller",
        default_value="true",
        description="Enable headless mode for robot control",
    )

    declare_initial_joint_controller = DeclareLaunchArgument(
        "initial_joint_controller",
        default_value=[tf_prefix, "joint_trajectory_controller"],
        description="Robot controller to start.",
    )

    declare_controllers_file = DeclareLaunchArgument(
        "controllers_file",
        default_value="kinova_controllers.yaml",
        description="YAML file with the controllers configuration.",
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            [tf_prefix, "joint_trajectory_controller"],
            "-t",
            "joint_trajectory_controller/JointTrajectoryController",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "10",
            "--namespace",
            device_namespace,
            "--param-file",
            namespaced_initial_joint_controllers_path,
        ],
        namespace=robot_namespace,
    )

    robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            [tf_prefix, "robotiq_gripper_controller"],
            "-t",
            "position_controllers/GripperActionController",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "10",
            "--namespace",
            device_namespace,
            "--param-file",
            namespaced_initial_joint_controllers_path,
        ],
        namespace=robot_namespace,
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=gz_bridge_name,
        parameters=[{"config_file": namespaced_gz_bridge_config_path}],
        namespace=robot_namespace,
        output="screen",
    )


    return LaunchDescription(
        [
            declare_device_namespace,
            declare_robot_namespace,
            declare_tf_prefix,
            declare_start_joint_controller,
            declare_initial_joint_controller,
            declare_controllers_file,
            namespace_warn,
            initial_joint_controller_spawner_started,
            robot_hand_controller_spawner,
            gz_bridge,
            OpaqueFunction(function=fix_depth_image_tf),
        ]
    )
