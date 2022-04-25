#!/usr/bin/env python3
#coding=utf-8
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
import ikpy.chain
from math import pi
from Arm_Lib import Arm_Device
import numpy as np


class JointStatePub(Node):

    def __init__(self):
        super().__init__('joint_state_pub')
        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(JointState, 'joint_states', qos_profile)
        timer_period = 7.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.sbus = Arm_Device()
        self.joints=['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.goal_position=[0.0,0.0,0.0,0.0,0.0]
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.name = self.joints
        msg.position = self.iks()
        print('joints' ,msg.position)
        if self.i%2 == 0:
            self.publisher_.publish(msg)
        else:
            msg.position = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.position)
        self.i+=1
        

       

    
    def iks(self):
        self.dofbot = ikpy.chain.Chain.from_urdf_file('/home/udo/ros2_ws/src/dofbot/urdf/arm.urdf')
        T = self.dofbot.forward_kinematics([0] * 6)
        #print('\nTransformation Matrix:\n', T)
        joints = self.dofbot.inverse_kinematics([-0.10,0.15,0.10]) #x(rechts),y(vorne), z(hoehe)
                
        joints_list = joints.tolist() 
        for i in range (5): 
                joints_list[i] = joints_list[i+1]
        del joints_list[5]
                
        return joints_list
        


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePub()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()