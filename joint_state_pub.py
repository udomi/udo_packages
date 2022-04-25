#!/usr/bin/env python3
#coding=utf-8
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState


class JointStatePub(Node):

    def __init__(self):
        super().__init__('joint_state_pub')
        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(JointState, 'joint_states', qos_profile)
        timer_period = 7.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        msg.position = [0.0,-0.75,-0.9,-1.15,0.0]
        print(msg)
        if self.i%2 == 0:
            self.publisher_.publish(msg)
        else:
            msg.position = [0.0,0.0,0.0,0.0,0.0]
            self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.position)
        self.i+=1
        


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePub()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()