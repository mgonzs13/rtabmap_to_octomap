from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    cloud_map_topic = LaunchConfiguration("cloud_map_topic")
    cloud_map_topic_cmd = DeclareLaunchArgument(
        "cloud_map_topic",
        default_value="cloud_map",
        description="Name of the rtabmap cloud_map topic")

    octomap_server_node_cmd = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server_node",
        remappings=[("cloud_in", cloud_map_topic)]
    )

    octomap_to_gridmap_demo_node = Node(
        package="rtabmap_to_octomap",
        executable="octomap_to_gridmap_demo",
        name="octomap_to_gridmap_demo",
        output="screen"
    )

    ld = LaunchDescription()
    ld.add_action(cloud_map_topic_cmd)
    ld.add_action(octomap_server_node_cmd)
    ld.add_action(octomap_to_gridmap_demo_node)
    return ld
