import rclpy
from rclpy.node import Node
import socket
import struct
from geometry_msgs.msg import PoseStamped
from tqdm import tqdm
import time
import numpy as np

class MocapUDPReceiver(Node):
    def __init__(self):
        super().__init__('mocap_udp_receiver')

        self.declare_parameter('udp_port', 22222)
        self.declare_parameter('timer_period', 0.005)
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.udp_port))
        self.sock.settimeout(0.1)  # 设置超时，避免阻塞

        self.num_bodies = None
        self.len_data = None
        self.pose_publishers = {}  # key = body id, value = publisher

        self.tqdm_bar = None

        self.timer = self.create_timer(self.timer_period, self.read_udp_data)

        self.get_logger().info(f"Listening on UDP {self.udp_port}")

    def read_udp_data(self):
        try:
            data, _ = self.sock.recvfrom(8192)  # 接收最大 8192 字节的数据

            if not data:
                self.get_logger().warn("No data received.")
                return

            if self.tqdm_bar is None:
                self.tqdm_bar = tqdm(total=len(data), desc="Receiving Data", position=0, dynamic_ncols=True)

            # 更新进度条
            self.tqdm_bar.update(len(data))

            if self.num_bodies is None:
                self.len_data = len(data)
                if self.len_data % 14 != 0:
                    self.get_logger().error("Invalid packet size")
                    return
                self.num_bodies = self.len_data // 14
                self.get_logger().info(f"Detected {self.num_bodies} rigid bodies")

                for i in range(self.num_bodies):
                    topic_name = f'/mocap/body{i+1}/pose'
                    self.pose_publishers[i + 1] = self.create_publisher(PoseStamped, topic_name, 10)
                    self.get_logger().info(f"Publishing: {topic_name}")

            for i in range(self.num_bodies):
                offset = i * 14
                x, y, z, qx, qy, qz, qw = struct.unpack("hhhhhhh", data[offset:offset + 14])

                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "world"
                pose_msg.pose.position.x = x * 0.0005
                pose_msg.pose.position.y = y * 0.0005
                pose_msg.pose.position.z = z * 0.0005
                pose_msg.pose.orientation.x = qx * 0.001
                pose_msg.pose.orientation.y = qy * 0.001
                pose_msg.pose.orientation.z = qz * 0.001
                pose_msg.pose.orientation.w = qw * 0.001

                self.pose_publishers[i + 1].publish(pose_msg)

            self.tqdm_bar.close()

        except socket.timeout:
            self.get_logger().warn("No data received in the specified timeout")
        except Exception as e:
            self.get_logger().error(f"Error while processing UDP data: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = MocapUDPReceiver()  # 创建节点实例
    rclpy.spin(node)           # 保持节点运行
    node.destroy_node()        # 关闭节点
    rclpy.shutdown()           # 关闭 ROS2
