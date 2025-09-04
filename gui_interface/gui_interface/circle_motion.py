#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleMover(Node):
    def __init__(self):
        super().__init__('tb3_circle_mover')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.2
        self.pub.publish(msg)

    def stop(self):
        msg = Twist()  # all zeros
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CircleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
