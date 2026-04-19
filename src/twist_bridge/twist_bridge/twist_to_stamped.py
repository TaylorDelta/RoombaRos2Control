#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistToStamped(Node):
    def __init__(self):
        super().__init__('twist_to_stamped')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_fg',   # Foxglove output
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            TwistStamped,
            'diff_drive_controller/cmd_vel',      # Robot expects stamped here
            10
        )

    def callback(self, msg):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'  # adjust if needed
        stamped.twist = msg

        self.publisher.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
