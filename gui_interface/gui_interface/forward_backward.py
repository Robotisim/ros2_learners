#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ForwardBackward(Node):
    def __init__(self):
        super().__init__('tb3_forward_backward')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # FORWARD -> PAUSE1 -> BACKWARD -> PAUSE2 -> ...
        self.state = 'FORWARD'
        self.state_elapsed = 0.0
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(0.05, self.tick)

    def set_cmd(self, vx=0.0):
        msg = Twist()
        msg.linear.x = vx
        self.pub.publish(msg)

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        self.state_elapsed += dt

        speed = 0.5
        move_time = 1.0
        pause_time = 0.5

        if self.state == 'FORWARD':
            self.set_cmd(speed)
            if self.state_elapsed >= move_time:
                self.state, self.state_elapsed = 'PAUSE1', 0.0
                self.set_cmd(0.0)

        elif self.state == 'PAUSE1':
            if self.state_elapsed >= pause_time:
                self.state, self.state_elapsed = 'BACKWARD', 0.0

        elif self.state == 'BACKWARD':
            self.set_cmd(-speed)
            if self.state_elapsed >= move_time:
                self.state, self.state_elapsed = 'PAUSE2', 0.0
                self.set_cmd(0.0)

        elif self.state == 'PAUSE2':
            if self.state_elapsed >= pause_time:
                self.state, self.state_elapsed = 'FORWARD', 0.0

    def stop(self):
        self.set_cmd(0.0)

def main():
    rclpy.init()
    node = ForwardBackward()
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
