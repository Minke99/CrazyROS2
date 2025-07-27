# joystick_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')

        pygame.init()
        pygame.joystick.init()

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f'Joystick: {self.joystick.get_name()}')

        self.pub = self.create_publisher(Joy, '/joy', 10)
        self.timer = self.create_timer(0.02, self.publish_joy)  # 50 Hz

    def publish_joy(self):
        pygame.event.pump()

        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        hat = self.joystick.get_hat(0)
        axes.append(float(hat[0]))  # ← →
        axes.append(float(hat[1]))  # ↑ ↓

        buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]

        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = axes
        joy_msg.buttons = buttons
        self.pub.publish(joy_msg)


# joy_mapper
from sensor_msgs.msg import Joy

class JoystickMapper:
    def __init__(self, joy_name='PS4 Controller'):
        self.axes = []
        self.buttons = []

        self.mapping = self._get_mapping(joy_name)

    def update(self, joy_msg: Joy):
        self.axes = joy_msg.axes
        self.buttons = joy_msg.buttons
        self.hat = (int(joy_msg.axes[-2]), int(joy_msg.axes[-1]))

    def get_key(self, key):
        idx = self.mapping.get(key, -1)
        if idx == -1:
            return 0

        if 0 <= idx < 100:  # buttons
            return self.buttons[idx]

        elif 100 <= idx < 200:  # axes
            return self.axes[idx - 100]

        elif 200 <= idx < 300:  # hat
            hat_x = self.hat[0]
            hat_y = self.hat[1]
            if idx == 201:  # Up
                return 1 if hat_y > 0 else 0
            elif idx == 202:  # Down
                return 1 if hat_y < 0 else 0
            elif idx == 203:  # Left
                return 1 if hat_x < 0 else 0
            elif idx == 204:  # Right
                return 1 if hat_x > 0 else 0
        return 0


    def _get_mapping(self, name):
        if name == 'PS4 Controller':
            return {
                'Cross': 0,
                'Circle': 1,
                'Triangle': 2,
                'Square': 3,
                'Share': 4,
                'Option': 6,
                'L1': 9,
                'R1': 10,
                'LeftStickX': 100,    # axes[0]
                'LeftStickY': 101,    # axes[1]
                'L2': 102,   # axes[2]
                'RightStickX': 103,            # axes[3]
                'RightStickY': 104,            # axes[4]
                'R2': 105,   # axes[5]
                'Touchpad': 15,
                'Up': 201,
                'Down': 202,
                'Left': 203,
                'Right': 204,
            }
        else:
            print(f"Unknown joystick '{name}', using default mapping.")
            return {}


def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()
