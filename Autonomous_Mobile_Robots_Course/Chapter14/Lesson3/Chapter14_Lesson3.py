"""
Chapter14_Lesson3.py
Autonomous Mobile Robots (AMR) — Chapter 14 Lesson 3
Behavior Trees / State Machines for Navigation

This file provides:
- A minimal Behavior Tree (BT) framework (from scratch)
- A navigation-oriented BT example using a shared blackboard
- A comparable hierarchical finite-state machine (HFSM) example

No external dependencies required (standard library only).
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Dict, List, Optional, Any
import time


# -----------------------------
# Behavior Tree core
# -----------------------------

class Status(Enum):
    SUCCESS = auto()
    FAILURE = auto()
    RUNNING = auto()


Blackboard = Dict[str, Any]


class Node:
    """Abstract BT node."""
    def __init__(self, name: str):
        self.name = name

    def tick(self, bb: Blackboard) -> Status:
        raise NotImplementedError


class Condition(Node):
    def __init__(self, name: str, fn: Callable[[Blackboard], bool]):
        super().__init__(name)
        self.fn = fn

    def tick(self, bb: Blackboard) -> Status:
        return Status.SUCCESS if self.fn(bb) else Status.FAILURE


class Action(Node):
    """
    Action node with optional internal state.
    The action function returns one of: 'success', 'failure', 'running'.
    """
    def __init__(self, name: str, fn: Callable[[Blackboard], str]):
        super().__init__(name)
        self.fn = fn

    def tick(self, bb: Blackboard) -> Status:
        out = self.fn(bb).lower().strip()
        if out == "success":
            return Status.SUCCESS
        if out == "failure":
            return Status.FAILURE
        return Status.RUNNING


class Sequence(Node):
    """Ticks children left-to-right. Fails fast; succeeds only if all succeed."""
    def __init__(self, name: str, children: List[Node]):
        super().__init__(name)
        self.children = children
        self._i = 0  # memory for RUNNING semantics (Sequence with memory)

    def tick(self, bb: Blackboard) -> Status:
        while self._i < len(self.children):
            s = self.children[self._i].tick(bb)
            if s == Status.SUCCESS:
                self._i += 1
                continue
            if s == Status.FAILURE:
                self._i = 0
                return Status.FAILURE
            return Status.RUNNING
        self._i = 0
        return Status.SUCCESS


class Fallback(Node):
    """Ticks children left-to-right. Succeeds fast; fails only if all fail."""
    def __init__(self, name: str, children: List[Node]):
        super().__init__(name)
        self.children = children
        self._i = 0  # memory for RUNNING semantics (Fallback with memory)

    def tick(self, bb: Blackboard) -> Status:
        while self._i < len(self.children):
            s = self.children[self._i].tick(bb)
            if s == Status.FAILURE:
                self._i += 1
                continue
            if s == Status.SUCCESS:
                self._i = 0
                return Status.SUCCESS
            return Status.RUNNING
        self._i = 0
        return Status.FAILURE


class RateLimiter(Node):
    """Ticks child at most once every dt seconds (simple 'decorator' node)."""
    def __init__(self, name: str, child: Node, dt: float):
        super().__init__(name)
        self.child = child
        self.dt = float(dt)
        self._t_last = 0.0
        self._last_status = Status.FAILURE

    def tick(self, bb: Blackboard) -> Status:
        now = time.time()
        if now - self._t_last >= self.dt:
            self._t_last = now
            self._last_status = self.child.tick(bb)
        return self._last_status


# -----------------------------
# Navigation-oriented primitives
# -----------------------------

def cond_have_goal(bb: Blackboard) -> bool:
    return bb.get("goal") is not None

def cond_goal_reached(bb: Blackboard) -> bool:
    # Goal is reached if distance <= tol
    return bb.get("dist_to_goal", float("inf")) <= bb.get("goal_tol", 0.2)

def cond_localization_ok(bb: Blackboard) -> bool:
    # "Localization OK" if covariance trace below a threshold
    P = bb.get("P_trace", 1e9)
    return P <= bb.get("P_trace_max", 2.0)

def cond_path_valid(bb: Blackboard) -> bool:
    # A coarse validity bit (e.g., global path not blocked)
    return bool(bb.get("path_valid", False))


def act_global_plan(bb: Blackboard) -> str:
    """
    Mock global planning:
    - Requires localization OK and a goal.
    - Produces a 'path' and marks path_valid.
    """
    if bb.get("goal") is None:
        return "failure"
    if not cond_localization_ok(bb):
        return "failure"
    # pretend planning succeeds with high probability
    bb["path"] = ["(x0,y0)", "(x1,y1)", "(xg,yg)"]
    bb["path_valid"] = True
    bb.setdefault("planner_calls", 0)
    bb["planner_calls"] += 1
    return "success"


def act_local_control(bb: Blackboard) -> str:
    """
    Mock local control:
    - If no valid path, fail.
    - Otherwise, reduce distance-to-goal.
    - Can 'stall' if obstacle flag is set.
    """
    if not bb.get("path_valid", False):
        return "failure"
    if bb.get("obstacle_blocking", False):
        # local controller running but not making progress
        bb["dist_to_goal"] = bb.get("dist_to_goal", 5.0)
        return "running"

    # progress
    d = float(bb.get("dist_to_goal", 5.0))
    v = float(bb.get("progress_per_tick", 0.15))
    bb["dist_to_goal"] = max(0.0, d - v)
    return "running" if not cond_goal_reached(bb) else "success"


def act_recover_clear_costmap(bb: Blackboard) -> str:
    """
    Mock recovery: clears obstacle and invalidates path (forces re-plan).
    """
    bb["obstacle_blocking"] = False
    bb["path_valid"] = False
    bb.setdefault("recoveries", 0)
    bb["recoveries"] += 1
    return "success"


def build_navigation_bt() -> Node:
    """
    A typical pattern:
    - Preconditions: have goal, localization ok
    - Plan at limited rate
    - Control until goal reached
    - If control stalls (obstacle), run recovery then re-plan
    """
    return Sequence("NavigateToGoal", [
        Condition("HaveGoal?", cond_have_goal),
        Condition("LocalizationOK?", cond_localization_ok),
        RateLimiter("Replan@1Hz", Action("GlobalPlan", act_global_plan), dt=1.0),
        Fallback("DriveOrRecover", [
            Sequence("DriveToGoal", [
                Condition("PathValid?", cond_path_valid),
                Action("LocalControl", act_local_control),
                Condition("GoalReached?", cond_goal_reached),
            ]),
            Action("RecoveryClearCostmap", act_recover_clear_costmap),
        ])
    ])


# -----------------------------
# Comparable Hierarchical FSM (HFSM)
# -----------------------------

class NavState(Enum):
    IDLE = auto()
    PLAN = auto()
    CONTROL = auto()
    RECOVERY = auto()
    DONE = auto()
    FAIL = auto()


@dataclass
class HFSM:
    state: NavState = NavState.IDLE
    stall_ticks: int = 0
    stall_max: int = 10

    def step(self, bb: Blackboard) -> NavState:
        if self.state == NavState.IDLE:
            if cond_have_goal(bb) and cond_localization_ok(bb):
                self.state = NavState.PLAN
            else:
                self.state = NavState.FAIL
            return self.state

        if self.state == NavState.PLAN:
            if act_global_plan(bb) == "success":
                self.state = NavState.CONTROL
            else:
                self.state = NavState.FAIL
            return self.state

        if self.state == NavState.CONTROL:
            if cond_goal_reached(bb):
                self.state = NavState.DONE
                return self.state
            out = act_local_control(bb)
            if out == "failure":
                self.state = NavState.PLAN
                return self.state
            # detect stall (no distance decrease) by obstacle flag
            if bb.get("obstacle_blocking", False):
                self.stall_ticks += 1
                if self.stall_ticks >= self.stall_max:
                    self.state = NavState.RECOVERY
                    self.stall_ticks = 0
            else:
                self.stall_ticks = 0
            return self.state

        if self.state == NavState.RECOVERY:
            act_recover_clear_costmap(bb)
            self.state = NavState.PLAN
            return self.state

        return self.state


# -----------------------------
# Demo
# -----------------------------

def run_demo():
    bb: Blackboard = {
        "goal": (5.0, 0.0),
        "dist_to_goal": 5.0,
        "goal_tol": 0.2,
        "P_trace": 0.8,
        "P_trace_max": 2.0,
        "progress_per_tick": 0.2,
        "obstacle_blocking": False,
        "path_valid": False,
    }

    bt = build_navigation_bt()
    fsm = HFSM()

    print("=== Behavior Tree demo ===")
    for t in range(60):
        # inject an obstacle episode
        if t == 12:
            bb["obstacle_blocking"] = True
        if t == 25:
            bb["obstacle_blocking"] = False

        s = bt.tick(bb)
        print(f"t={t:02d} status={s.name:7s} dist={bb['dist_to_goal']:.2f} "
              f"path_valid={bb['path_valid']} rec={bb.get('recoveries',0)}")
        if s == Status.SUCCESS:
            break
        time.sleep(0.02)

    # reset
    bb["dist_to_goal"] = 5.0
    bb["path_valid"] = False
    bb["obstacle_blocking"] = False
    bb["recoveries"] = 0

    print("\n=== HFSM demo ===")
    for t in range(60):
        if t == 12:
            bb["obstacle_blocking"] = True
        if t == 25:
            bb["obstacle_blocking"] = False

        st = fsm.step(bb)
        print(f"t={t:02d} state={st.name:9s} dist={bb['dist_to_goal']:.2f} "
              f"path_valid={bb['path_valid']} rec={bb.get('recoveries',0)}")
        if st == NavState.DONE:
            break
        time.sleep(0.02)


if __name__ == "__main__":
    run_demo()
