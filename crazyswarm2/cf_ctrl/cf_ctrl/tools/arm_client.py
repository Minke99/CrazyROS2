#!/usr/bin/env python3
"""
arm_client.py
最小化 Crazyflie Arm 封装
"""

import rclpy
from rclpy.node import Node
from crazyflie_interfaces.srv import Arm


class ArmClient:
    """只负责 Arm 服务的简洁类"""

    def __init__(self, node_name: str = "arm_client"):
        # 只做一次 rclpy 初始化，方便在脚本或主节点中重复实例化
        if not rclpy.ok():
            rclpy.init()
        self._node = Node(node_name)
        self._clients = {}  # {"<ns>/arm": Client}

    def arm(self, namespace: str, state: bool, timeout: float = 5.0) -> bool:
        """
        解锁或上锁 Crazyflie／Bolt

        参数
        ----
        namespace:  "/cf231"  或 "/all"
        state:      True=解锁   False=上锁
        timeout:    等待服务可用的秒数
        返回
        ----
        bool: 调用是否成功
        """
        srv = f"{namespace.rstrip('/')}/arm"

        client = self._clients.get(srv)
        if client is None:
            client = self._node.create_client(Arm, srv)
            if not client.wait_for_service(timeout_sec=timeout):
                self._node.get_logger().error(f"找不到服务 {srv}")
                return False
            self._clients[srv] = client

        req = Arm.Request()
        req.arm = bool(state)

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)

        if future.result() is None:
            self._node.get_logger().error(f"调用 {srv} 失败: {future.exception()}")
            return False

        return True

    def close(self):
        """销毁节点并关闭 rclpy"""
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
