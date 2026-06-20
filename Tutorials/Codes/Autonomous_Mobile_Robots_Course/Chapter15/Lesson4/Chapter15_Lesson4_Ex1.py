# Chapter15_Lesson4_Ex1.py
# Exercise: visualize the optimized TEB trajectory and inspect constraint violations.

import numpy as np
import matplotlib.pyplot as plt

from Chapter15_Lesson4 import optimize_teb_demo


def main():
    out = optimize_teb_demo()
    xs, ys = out["xs"], out["ys"]
    obs = out["obstacles"]
    start = out["start"]
    goal = out["goal"]

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(xs, ys, marker="o", linewidth=2, label="TEB optimized")
    ax.plot([start[0]], [start[1]], marker="s", markersize=10, label="start")
    ax.plot([goal[0]], [goal[1]], marker="*", markersize=12, label="goal")

    for o in obs:
        circ = plt.Circle((o.cx, o.cy), o.r, fill=False)
        ax.add_patch(circ)

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.set_title("Timed-Elastic-Band local planning (demo)")
    ax.legend()
    plt.show()


if __name__ == "__main__":
    main()
