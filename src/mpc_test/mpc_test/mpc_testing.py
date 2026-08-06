#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped

 

class MpcTest(Node):
    def __init__(self):
        super().__init__('mpc_testing')


        self.publisher = self.create_publisher(
            TwistStamped,
            '/roomba_controller/cmd_vel',      # roomba_controller expects TwistStamped here
            10
        )

        self.count = 0
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.send_vel_cmd)

        self.v = [5.01800630e-37, 1.87499988e-02, 3.74999983e-02, 5.62499980e-02, 7.49999977e-02, 9.37499974e-02, 1.12499997e-01, 1.31249996e-01, 1.49999995e-01, 1.68749992e-01, 1.87499981e-01, 1.97737315e-01, 1.95021696e-01, 1.83032915e-01, 1.65037363e-01, 1.46287396e-01, 1.27537418e-01, 1.08787439e-01, 9.00374605e-02, 7.12874806e-02, 5.25374964e-02, 3.37875083e-02, 1.50375386e-02, -1.49985904e-03, -1.61816700e-02, -3.49316521e-02, -5.36816403e-02, -7.24316285e-02, -9.11816161e-02, -1.09931602e-01, -1.28681584e-01, -1.32100665e-01, -1.13350672e-01, -9.46006746e-02, -7.58506777e-02, -5.71006821e-02, -3.83506916e-02, -2.24968396e-02, -2.17179702e-02, -2.96898962e-02, -4.48649997e-02, -6.36149680e-02, -8.23649527e-02, -1.01114783e-01, -1.15332777e-01, -1.25227601e-01, -1.32090830e-01, -1.34947179e-01, -1.33003303e-01, -1.26616611e-01, -1.16733440e-01]
        self.omega = [0.0, 0.49999998, 0.49999998, 0.49999998, 0.49999998, 0.49999998, 0.49999996, 0.49999994, 0.49999984, 0.44645975, -0.19468599, -0.38121365, -0.48032901, -0.49999773, -0.49999781, -0.45394927, -0.2656778, -0.08121759, 0.06231101, 0.16651647, 0.23083333, 0.23197062, 0.06149822, 0.01322432, 0.14863514, 0.34830355, -0.19928732, -0.37783478, -0.41286188, -0.43616692, -0.46106066, -0.03080641, -0.1995995, -0.49999946, -0.49999988, -0.4999999, -0.4999999, -0.4999999, -0.4999999, -0.4999999, -0.4999999, -0.49999989, -0.49999983, -0.49999978, -0.4999995, -0.49999877, -0.18932145, 0.0334553, -0.28636483, -0.44035568, -0.47875624]



    def send_vel_cmd(self):

        if self.count < len(self.v):
            out = TwistStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'base_link'
            out.twist.linear.x = float(self.v[self.count])
            out.twist.angular.z = float(self.omega[self.count])
            self.publisher.publish(out)
    
        self.get_logger().info(f'Publishing: "{out.data}"')

        self.count += 1



def main(args=None):
    rclpy.init(args=args)
    node = MpcTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
