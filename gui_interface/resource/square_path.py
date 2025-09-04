#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SquarePath(Node):
    def __init__(self):
        super().__init__('tb3_square_path')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
         # DRIVE -> TURN -> ...
        self.state = 'DRIVE'
        self.state_elapsed = 0.0
        self.last_time = self.get_clock().now()

        self.update_durations()

        self.timer = self.create_timer(0.05, self.tick)

    def update_durations(self):
        edge = 1.0
        v = 0.25
        w = 0.5
        ang = math.pi/2

        self.drive_time = abs(edge / v) if v != 0 else 0.0
        self.turn_time = abs(ang / w) if w != 0 else 0.0

    def cmd(self, vx=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.pub.publish(msg)

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        self.state_elapsed += dt

        v = 0.25
        w = 0.5
        if self.state == 'DRIVE':
            self.cmd(vx=v, wz=0.0)
            if self.state_elapsed >= self.drive_time:
                self.state, self.state_elapsed = 'TURN', 0.0
                self.cmd(0.0, 0.0)
        elif self.state == 'TURN':
            self.cmd(vx=0.0, wz=w)
            if self.state_elapsed >= self.turn_time:
                self.state, self.state_elapsed = 'DRIVE', 0.0
                self.cmd(0.0, 0.0)

    def stop(self):
        self.cmd(0.0, 0.0)

def main():
    rclpy.init()
    node = SquarePath()
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
