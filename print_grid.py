
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from grid_map_msgs.msg import GridMap


def print_grid(msg: GridMap) -> None:
    print(msg.info)
    print()
    print(msg.layers)
    print(msg.basic_layers)
    print()
    print(msg.data[0])
    print()
    print(msg.data[1])


rclpy.init()

qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=1
)

node = Node("print_grind_node")
node.create_subscription(GridMap, "/grid_map", print_grid, qos_profile)

rclpy.spin(node)

rclpy.shutdown()
