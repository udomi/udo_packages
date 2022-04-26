#!/usr/bin/env python3
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped



class ObjectFramePublisher(Node):

    def __init__(self):
        super().__init__('tf2_object_frame_publisher')

        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        # callback function on each message
        self.subscription = self.create_subscription(PoseStamped, '/object', self.handle_object_pose, 1)
        self.subscription

    def handle_object_pose(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera'
        t.child_frame_id = msg.header.frame_id


        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        # Send the transformation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = ObjectFramePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()