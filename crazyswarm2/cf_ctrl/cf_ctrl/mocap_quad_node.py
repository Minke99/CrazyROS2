import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

# for differentiating data
from collections import deque
import math
from cf_ctrl.joystick_3 import PygameJoystick
from cf_ctrl.IIR2Filter import IIR2Filter
from cf_ctrl.standard_pid import PidControlRaw
from scipy.spatial.transform import Rotation

class Differentiator:
    def __init__(self, diff_steps=1):
        self.t_queue = deque([0] * diff_steps, maxlen=diff_steps)
        self.data_queue = deque([0] * diff_steps, maxlen=diff_steps)
        self.data_rate = 0

    def step(self, data_now, abstime):
        dt = abstime - self.t_queue[0]
        if dt == 0:
            self.t_queue.append(abstime)
        else:
            self.data_rate = (data_now - self.data_queue[0]) / dt
            self.t_queue.append(abstime)
            self.data_queue.append(data_now)

        return self.data_rate

class RiseDetect:
    def __init__(self, ):
        self.flag = False
        self.flag_old = False
        self.force_enable_flag = False

    def step(self, input):
        self.flag_old = self.flag
        self.flag = input

        if self.force_enable_flag:
            self.force_enable_flag = False
            return True
        else:
            if self.flag and not self.flag_old:
                return True
            else:
                return False

    def force_enable(self):
        self.force_enable_flag = True

def saturation(x, max_x, min_x):
    if x > max_x:
        return max_x
    elif x < min_x:
        return min_x
    else:
        return x
    
class Controller(Node):
    def __init__(self):
        super().__init__('mocap_ctrl_node')

        self.subscription = self.create_subscription(
            PoseStamped,
            '/mocap/body1/pose',
            self.mocap_pose,
            10
        )

        self.cmd_publishers = self.create_publisher(
            Twist, 
            '/cf231/cmd_vel_legacy', 
            10
        )

        self.mocap_time = 0.0
        self.mocap_x = 0.0
        self.mocap_y = 0.0
        self.mocap_z = 0.0
        self.mocap_qx = 0.0
        self.mocap_qy = 0.0
        self.mocap_qz = 0.0
        self.mocap_qw = 1.0
        print('mocap pose subscriber initialized')
        self.PJ = PygameJoystick()
        print('Joystick initialized')

        if True:  # Replace with actual condition to check if joystick is available
            self.PID_x_flight = PidControlRaw(20, 0, 5, 0, 0, 0)
            self.PID_y_flight = PidControlRaw(20, 0, 5, 0, 0, 0)
            self.PID_z_flight = PidControlRaw(10000, 0, 10000, 18000, 0, 0)
            self.roll_flight, self.pitch_flight, self.yaw_flight = 0, 0, 0
            self.thrust_flight = 0
        
        # reference
        if True:
            self.desired_x, self.desired_y, self.desired_z = 0.0, 0.0, 0.65
            self.desired_yaw = 0

        # ctrl loop
        self.freq = 100
        timer_period = 1/self.freq
        self.timer = self.create_timer(timer_period, self.ctrl_loop)

            # Filter and Differentiator
        if True:
            self.Filter_x = IIR2Filter(4, [10], 'lowpass', design='cheby2', rs=5, fs=self.freq)
            self.Filter_y = IIR2Filter(4, [10], 'lowpass', design='cheby2', rs=5, fs=self.freq)
            self.Filter_z = IIR2Filter(4, [10], 'lowpass', design='cheby2', rs=5, fs=self.freq)
            self.Diff_X = Differentiator(diff_steps=2)
            self.Diff_Y = Differentiator(diff_steps=2)
            self.Diff_Z = Differentiator(diff_steps=2)

        # buttons and system control
        self.RD_circle_button = RiseDetect()
        self.controller_start_flag = False

        self.RD_high_button = RiseDetect()
        self.RD_low_button = RiseDetect()

        self.extpose_waiting_cycles = 10
        self.loop_count = 0
        

        self.get_logger().info("Mocap ctrl node started")

    def mocap_pose(self, msg):
        self.mocap_x = msg.pose.position.x
        self.mocap_y = msg.pose.position.y
        self.mocap_z = msg.pose.position.z
        self.mocap_qx = msg.pose.orientation.x
        self.mocap_qy = msg.pose.orientation.y
        self.mocap_qz = msg.pose.orientation.z
        self.mocap_qw = msg.pose.orientation.w
        self.mocap_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # print(f'Mocap pose received: x={self.mocap_x}, y={self.mocap_y}, z={self.mocap_z}, qx={self.mocap_qx}, qy={self.mocap_qy}, qz={self.mocap_qz}, qw={self.mocap_qw}')

    def ctrl_loop(self):
        # code
        self.loop_count += 1
        Abs_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        self.PJ.step()

        if True:
            robot_R = Rotation.from_quat([self.mocap_qx, self.mocap_qy, self.mocap_qz, self.mocap_qw])
            self.robot_R_euler = robot_R.as_euler('xyz', degrees=True)

        # filter and differentiate
        if True:
            # self.mocap_pose()
            self.mocap_x_f = self.Filter_x.filter(self.mocap_x)
            self.mocap_y_f = self.Filter_y.filter(self.mocap_y)
            self.mocap_z_f = self.Filter_z.filter(self.mocap_z)

            self.Diff_X.step(self.mocap_x_f, self.mocap_time)
            self.Diff_Y.step(self.mocap_y_f, self.mocap_time)
            self.Diff_Z.step(self.mocap_z_f, self.mocap_time)

        # set reference (X,Y,height)
        if True:
            JSK_x, JSK_y = self.PJ.get_key('LeftStickY'), self.PJ.get_key('LeftStickX')
            if abs(JSK_x) < 0.2:
                JSK_x = 0
            if abs(JSK_y) < 0.2:
                JSK_y = 0
            self.desired_x = JSK_x * self.freq + self.desired_x
            self.desired_y = JSK_y * self.freq + self.desired_y

            if self.RD_high_button.step(self.PJ.get_key('Triangle')):
                self.desired_z = self.desired_z + 0.1
            if self.RD_low_button.step(self.PJ.get_key('Square')):
                self.desired_z = self.desired_z - 0.1

            JS_x = self.PJ.get_key('RightStickX')
            if abs(JS_x) < 0.2:
                JS_x = 0
            JS_x = JS_x * 30 * self.freq
            self.desired_yaw = self.desired_yaw + JS_x
            if self.desired_yaw > 180:
                self.desired_yaw = self.desired_yaw - 360
            if self.desired_yaw < - 180:
                self.desired_yaw = self.desired_yaw + 360

        # buttons and system control
        if self.RD_circle_button.step(self.PJ.get_key('Circle')):
            if self.controller_start_flag:
                self.controller_start_flag = False
                print('controller stop')
            else:
                self.controller_start_flag = True
                print('controller start')
        
         # flight controller
        if True:
            self.PID_x_flight.update_reference(self.desired_x, 0)
            self.PID_y_flight.update_reference(self.desired_y, 0)
            self.PID_z_flight.update_reference(self.desired_z, 0)

            u_x = self.PID_x_flight.update_error(self.mocap_x_f, self.Diff_X.data_rate, Abs_time)
            u_y = self.PID_y_flight.update_error(self.mocap_y_f, self.Diff_Y.data_rate, Abs_time)
            u_z = self.PID_z_flight.update_error(self.mocap_z_f, self.Diff_Z.data_rate, Abs_time)
            angle_yaw = self.robot_R_euler[0]

            self.pitch_flight = (u_x * math.cos(angle_yaw) + u_y * math.sin(angle_yaw))
            self.roll_flight = -(u_y * math.cos(angle_yaw) - u_x * math.sin(angle_yaw))

            self.pitch_flight = saturation(self.pitch_flight, 25, -25)
            self.roll_flight = saturation(self.roll_flight, 25, -25)
            self.thrust_flight = round(saturation(u_z, 34000, 3000))
            

            Yaw_error = self.desired_yaw - angle_yaw
            if Yaw_error > math.pi:
                Yaw_error = Yaw_error - 2 * math.pi
            elif Yaw_error < -math.pi:
                Yaw_error = Yaw_error + 2 * math.pi
            self.yaw_flight = - Yaw_error * 30
        
        if True:  # Replace with actual condition to check if joystick is available
            if self.controller_start_flag:
                # cmd_pitch = self.pitch_flight
                # cmd_roll = self.roll_flight
                # cmd_yaw = self.yaw_flight
                # cmd_thrust = self.thrust_flight
                cmd_pitch = 0.0
                cmd_roll = 0.0
                cmd_yaw = 0.0
                cmd_thrust = 30000.0
                
            else:
                cmd_pitch = 0.0
                cmd_roll = 0.0
                cmd_yaw = 0.0
                cmd_thrust = 0.0

        if True:
            msg = Twist()
            msg.linear.x = cmd_pitch
            msg.linear.y = cmd_roll
            msg.linear.z = cmd_thrust
            msg.angular.z = cmd_yaw
            self.cmd_publishers.publish(msg)

def main(args=None):
    print('Starting mocap_quad_node...')
    rclpy.init(args=args)
    print('rclpy initialized')
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
