#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class AmclPoseListener(Node):
    def __init__(self):
        super().__init__('amcl_pose_listener')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot4/amcl_pose',
            self.pose_callback,
            10
        )
        self.get_logger().info('✅ Subscribed to /robot4/amcl_pose')

    def pose_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.get_logger().info(
            f'📍 Position: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f} | '
            f'Orientation (quaternion): x={orientation.x:.2f}, y={orientation.y:.2f}, z={orientation.z:.2f}, w={orientation.w:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AmclPoseListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
