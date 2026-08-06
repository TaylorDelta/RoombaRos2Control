#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistToStamped(Node):
    def __init__(self):
        super().__init__('twist_to_stamped')

        # Declare and get namespace parameter (e.g., 'robot1', 'robot2')
        self.declare_parameter('namespace', '')
        namespace = self.get_parameter('namespace').get_parameter_value().string_value
        
        # Add leading slash if namespace is provided and doesn't have one
        if namespace and not namespace.startswith('/'):
            namespace = '/' + namespace
        
        # Build topic names with namespace
        input_topic = f'{namespace}/cmd_vel_fg'
        output_topic = f'{namespace}/diff_drive_controller/cmd_vel'

        self.subscription = self.create_subscription(
            Twist,
            input_topic,
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            TwistStamped,
            output_topic,
            10
        )
        
        self.get_logger().info(f'TwistToStamped node started with namespace: "{namespace or "global"}"')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing to: {output_topic}')

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
