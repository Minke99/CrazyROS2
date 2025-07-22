import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class CrazyflieCtrlNode(Node):
    def __init__(self):
        super().__init__('cf_ctrl_node')



def main():
    rclpy.init()
    node = CrazyflieCtrlNode()
    rclpy.spin(node)
    rclpy.shutdown()
