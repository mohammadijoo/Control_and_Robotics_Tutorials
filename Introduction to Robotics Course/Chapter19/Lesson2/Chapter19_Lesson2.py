import collections
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import matplotlib.pyplot as plt

WINDOW_SIZE = 200

class JointPlotNode(Node):
    def __init__(self):
        super().__init__("joint_plot_node")
        self.buffer_t = collections.deque(maxlen=WINDOW_SIZE)
        self.buffer_q = collections.deque(maxlen=WINDOW_SIZE)
        self.sub = self.create_subscription(
            JointState, "/joint_states", self.callback, 10
        )

    def callback(self, msg):
        # assume first joint
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = msg.position[0]
        self.buffer_t.append(t)
        self.buffer_q.append(q)

def plotting_thread(node):
    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [])
    ax.set_xlabel("time [s]")
    ax.set_ylabel("joint position [rad]")

    while rclpy.ok():
        if node.buffer_t:
            t0 = node.buffer_t[0]
            t_vals = [ti - t0 for ti in node.buffer_t]
            q_vals = list(node.buffer_q)
            line.set_xdata(t_vals)
            line.set_ydata(q_vals)
            ax.relim()
            ax.autoscale_view()
            plt.draw()
            plt.pause(0.01)
        else:
            plt.pause(0.01)

def main():
    rclpy.init()
    node = JointPlotNode()

    th = threading.Thread(target=plotting_thread, args=(node,), daemon=True)
    th.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
      
