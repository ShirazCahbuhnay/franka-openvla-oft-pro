#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger

class Ps5ArmHomeNode(Node):
    def __init__(self) -> None:
        super().__init__('ps5_arm_home_node')

        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('require_enable_button', True)
        self.declare_parameter('enable_button', 9)
        self.declare_parameter('home_button', 11)
        self.declare_parameter('home_axis', 7)
        self.declare_parameter('home_axis_sign', 1.0)
        self.declare_parameter('command_cooldown_sec', 1.0)
        
        self.joy_topic = str(self.get_parameter('joy_topic').value)
        self.require_enable_button = bool(self.get_parameter('require_enable_button').value)
        self.enable_button = int(self.get_parameter('enable_button').value)
        
        self.home_button = int(self.get_parameter('home_button').value)
        self.home_axis = int(self.get_parameter('home_axis').value)
        self.home_axis_sign = float(self.get_parameter('home_axis_sign').value)
        self.command_cooldown_sec = float(self.get_parameter('command_cooldown_sec').value)
        
        self.previous_button_active = False
        self.last_command_time = 0.0

        # Publisher to the joint trajectory controller
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, 
            '/panda_arm_controller/joint_trajectory', 
            10
        )
        
        # Subscribe to joy
        self.subscription = self.create_subscription(
            Joy,
            self.joy_topic,
            self.joy_callback,
            10,
        )

        # Service clients for MoveIt Servo
        self.stop_client = self.create_client(Trigger, '/servo_node/stop_servo')
        self.start_client = self.create_client(Trigger, '/servo_node/start_servo')

        # Standard Franka ready joint positions (radians)
        self.home_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]

        self.get_logger().info(
            f'Arm homing node ready: hold enable ({self.enable_button}), '
            f'press home button ({self.home_button}) or axis ({self.home_axis})'
        )

    def joy_callback(self, joy_msg: Joy) -> None:
        if self.require_enable_button and not self._button_pressed(joy_msg, self.enable_button):
            self.previous_button_active = False
            return
            
        current_active = self._home_active(joy_msg)
        
        # Rising edge detection
        if current_active and not self.previous_button_active:
            if self._command_allowed():
                self.get_logger().info("Homing command triggered! Stopping servo and sending arm to ready position...")
                self.execute_homing_sequence()
                self.last_command_time = time.monotonic()
                
        self.previous_button_active = current_active

    def _home_active(self, joy_msg: Joy) -> bool:
        # Check button
        if self._button_pressed(joy_msg, self.home_button):
            return True
            
        # Check axis
        if self.home_axis >= 0 and self.home_axis < len(joy_msg.axes):
            val = joy_msg.axes[self.home_axis]
            if self.home_axis_sign > 0 and val > 0.5:
                return True
            if self.home_axis_sign < 0 and val < -0.5:
                return True
                
        return False

    def _button_pressed(self, joy_msg: Joy, button_index: int) -> bool:
        if button_index < 0 or button_index >= len(joy_msg.buttons):
            return False
        return joy_msg.buttons[button_index] == 1

    def _command_allowed(self) -> bool:
        # Allow enough time for homing to complete (e.g. 5 seconds)
        elapsed = time.monotonic() - self.last_command_time
        return elapsed >= 5.0

    def execute_homing_sequence(self) -> None:
        # 1. Stop Servo
        if self.stop_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            self.stop_client.call_async(req)
            self.get_logger().info("Stopped MoveIt Servo.")
        else:
            self.get_logger().warn("Stop servo service not available! Proceeding anyway...")

        # 2. Send Trajectory
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.home_joints
        point.time_from_start.sec = 4  # 4 seconds transition
        
        traj_msg.points.append(point)
        self.trajectory_pub.publish(traj_msg)
        self.get_logger().info("Trajectory published.")

        # 3. Wait for trajectory to finish, then start
        # Create a one-shot timer to start servo after the trajectory is done
        self.start_timer = self.create_timer(4.5, self._start_servo_callback)

    def _start_servo_callback(self):
        # Cancel the timer so it only runs once
        if self.start_timer:
            self.start_timer.cancel()
            
        if self.start_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            self.start_client.call_async(req)
            self.get_logger().info("Started MoveIt Servo.")
        else:
            self.get_logger().warn("Start servo service not available!")

def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ps5ArmHomeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
