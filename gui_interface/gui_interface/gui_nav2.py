#!/usr/bin/env python3
import sys, os, signal, subprocess, threading, time
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton
from PyQt5.QtCore import QTimer

procs = {}

def _sh(cmd):
    return subprocess.Popen(
        f'bash -lc "{cmd}"',
        shell=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )

def launch_tb3_sim_and_nav2():
    gazebo_cmd = "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
    nav2_cmd = "ros2 launch turtlebot3_navigation2 navigation2.launch.py"
    procs['gazebo'] = _sh(gazebo_cmd)
    procs['nav2'] = _sh(nav2_cmd)

def stop_all():
    for p in list(procs.values()):
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
            time.sleep(0.3)
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass
    procs.clear()

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class WaypointNode(Node):
    def __init__(self):
        super().__init__('tb3_gui_min')
        self.latest_amcl = None
        self.waypoints = []
        self.navigator = BasicNavigator()
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)

    def _amcl_cb(self, msg):
        self.latest_amcl = msg

    def save_pose(self):
        if self.latest_amcl is None:
            return
        ps = PoseStamped()
        ps.header = self.latest_amcl.header
        ps.pose = self.latest_amcl.pose.pose
        self.waypoints.append(ps)

    def go_to_saved(self):
        if not self.waypoints:
            return
        def run():
            self.navigator.waitUntilNav2Active()
            self.navigator.followWaypoints(self.waypoints)
            while not self.navigator.isTaskComplete():
                time.sleep(0.2)
        threading.Thread(target=run, daemon=True).start()

def main():
    rclpy.init()
    node = WaypointNode()
    exec_ = SingleThreadedExecutor()
    exec_.add_node(node)

    app = QApplication(sys.argv)
    w = QWidget(); w.setWindowTitle("TB3 Minimal Launcher")
    lay = QVBoxLayout(w)

    b = QPushButton("Launch TB3 Sim + Nav2")
    b.clicked.connect(launch_tb3_sim_and_nav2); lay.addWidget(b)

    b = QPushButton("Save Pose")
    b.clicked.connect(node.save_pose); lay.addWidget(b)

    b = QPushButton("Go To Saved Poses")
    b.clicked.connect(node.go_to_saved); lay.addWidget(b)

    b = QPushButton("Stop All")
    b.clicked.connect(stop_all); lay.addWidget(b)

    timer = QTimer(w)
    timer.timeout.connect(lambda: exec_.spin_once(timeout_sec=0.0))
    timer.start(10)

    app.aboutToQuit.connect(lambda: (stop_all(),
                                     exec_.remove_node(node),
                                     node.destroy_node(),
                                     rclpy.shutdown()))

    w.setLayout(lay); w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
