#!/usr/bin/env python3
import sys, os, signal, subprocess
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton

procs = {}

CMDS = {
    "gazebo": ["ros2", "launch", "turtlebot3_gazebo", "empty_world.launch.py"],
    "circle_motion": ["ros2", "run", "gui_interface", "circle_motion"],
    "forward_backward": ["ros2", "run", "gui_interface", "forward_backward"],
    "square_path": ["ros2", "run", "gui_interface", "square_path"],
}

NODE_NAMES = ["circle_motion", "forward_backward", "square_path"]

def start(name):
    p = procs.get(name)
    if p and p.poll() is None:
        return
    procs[name] = subprocess.Popen(
        CMDS[name],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )

def stop(name):
    p = procs.get(name)
    if not p:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
        try:
            p.wait(timeout=2)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
    except Exception:
        pass
    procs.pop(name, None)

def close_all_nodes():
    for n in NODE_NAMES:
        stop(n)

def kill_gazebo_leftovers():
    for pat in ["gzserver", "gzclient", "gazebo", "ign gazebo", "ign-gazebo"]:
        subprocess.run(["pkill", "-f", pat], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, check=False)

def stop_all_nodes_and_gazebo():
    stop("gazebo")
    close_all_nodes()
    kill_gazebo_leftovers()

def main():
    app = QApplication(sys.argv)
    w = QWidget(); w.setWindowTitle("TB3 Minimal Launcher — Stage 3")
    lay = QVBoxLayout(w)

    btn = QPushButton("Launch Gazebo (TB3 Empty World)")
    btn.clicked.connect(lambda: start("gazebo")); lay.addWidget(btn)

    b = QPushButton("Start: circle_motion")
    b.clicked.connect(lambda: start("circle_motion")); lay.addWidget(b)

    b = QPushButton("Start: forward_backward")
    b.clicked.connect(lambda: start("forward_backward")); lay.addWidget(b)

    b = QPushButton("Start: square_path")
    b.clicked.connect(lambda: start("square_path")); lay.addWidget(b)

    b = QPushButton("Close All Nodes")
    b.clicked.connect(close_all_nodes); lay.addWidget(b)

    b = QPushButton("Stop All (Nodes + Gazebo)")
    b.clicked.connect(stop_all_nodes_and_gazebo); lay.addWidget(b)

    app.aboutToQuit.connect(stop_all_nodes_and_gazebo)

    w.setLayout(lay); w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
