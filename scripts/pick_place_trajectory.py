#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64MultiArray

class Demo(Node):
    def __init__(self):
        super().__init__('robomaster_pick_place_demo')
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)

    def arm(self, name, a, w, sec=3):
        msg = JointTrajectory()
        msg.joint_names = ['arm_lift_joint', 'wrist_pitch_joint']
        p = JointTrajectoryPoint()
        p.positions = [a, w]
        p.time_from_start = Duration(sec=sec)
        msg.points = [p]
        self.get_logger().info(f'{name}: arm_lift={a:.2f}, wrist={w:.2f}')
        self.arm_pub.publish(msg)
        time.sleep(sec + 0.8)

    def grip(self, name, v):
        self.get_logger().info(f'{name}: gripper={v:.3f}')
        self.grip_pub.publish(Float64MultiArray(data=[v, v]))
        time.sleep(1.2)

def main():
    rclpy.init()
    n = Demo()
    time.sleep(1.5)
    n.grip('open', 0.040)
    n.arm('home', 0.20, 0.00, 3)
    n.arm('above object', 0.45, -0.35, 4)
    n.arm('down to object', 0.18, -0.50, 4)
    n.grip('close on object', 0.006)
    n.arm('lift', 0.55, -0.35, 4)
    n.arm('place side', 0.55, 0.45, 5)
    n.arm('place down', 0.22, 0.35, 4)
    n.grip('release', 0.040)
    n.arm('return home', 0.20, 0.00, 4)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
