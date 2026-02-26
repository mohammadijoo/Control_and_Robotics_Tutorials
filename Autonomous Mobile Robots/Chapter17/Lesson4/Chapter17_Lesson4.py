# Chapter17_Lesson4.py
# Exploration Under Limited Battery/Time (Budget-Aware Frontier Exploration)
# Author: course material generator
#
# This script demonstrates a simple, self-contained budget-aware exploration loop:
# - A probabilistic occupancy grid with unknown cells
# - Frontier detection
# - Utility = expected information gain - lambda * travel_cost
# - Safety constraint: must have enough budget to reach goal and return home + reserve
#
# No ROS required; "integration points" are noted in comments.

from __future__ import annotations
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
import heapq
import math
import random

random.seed(7)

Cell = Tuple[int, int]

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def entropy_bern(p: float) -> float:
    """Binary entropy H(p) in nats."""
    p = clamp(p, 1e-9, 1 - 1e-9)
    return -(p * math.log(p) + (1 - p) * math.log(1 - p))

@dataclass
class Budget:
    # energy in Joules (or arbitrary units)
    E: float
    # time in seconds (or arbitrary units)
    T: float

@dataclass
class RobotParams:
    # cost per grid step (move energy) and per step time
    E_step: float = 1.0
    T_step: float = 1.0
    # idle/base costs per step
    E_base: float = 0.10
    # reserve to guarantee safe stop/return
    E_reserve: float = 10.0

@dataclass
class SensorParams:
    # sensing radius (Chebyshev radius in grid)
    r: int = 3
    # observation quality: new occupancy probability when observed obstacle
    p_occ_obs: float = 0.95
    p_free_obs: float = 0.05

class OccupancyGrid:
    """
    Probability grid p_occ in [0,1].
    Unknown cells are initialized to 0.5.
    """
    def __init__(self, w: int, h: int):
        self.w, self.h = w, h
        self.p = [[0.5 for _ in range(w)] for _ in range(h)]

    def inb(self, c: Cell) -> bool:
        x, y = c
        return 0 <= x < self.w and 0 <= y < self.h

    def get(self, c: Cell) -> float:
        x, y = c
        return self.p[y][x]

    def set(self, c: Cell, p_occ: float) -> None:
        x, y = c
        self.p[y][x] = clamp(p_occ, 0.0, 1.0)

    def is_free(self, c: Cell, thr: float = 0.65) -> bool:
        return self.get(c) < thr

    def is_occupied(self, c: Cell, thr: float = 0.65) -> bool:
        return self.get(c) > thr

    def neighbors4(self, c: Cell) -> List[Cell]:
        x, y = c
        cand = [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
        return [q for q in cand if self.inb(q)]

def astar(grid: OccupancyGrid, s: Cell, g: Cell, occ_thr: float = 0.65) -> Optional[List[Cell]]:
    """A* on 4-neighborhood; treats p_occ > occ_thr as blocked."""
    if not grid.inb(s) or not grid.inb(g):
        return None
    if not grid.is_free(s, occ_thr) or not grid.is_free(g, occ_thr):
        return None

    def h(c: Cell) -> float:
        return abs(c[0]-g[0]) + abs(c[1]-g[1])

    openpq: List[Tuple[float, Cell]] = []
    heapq.heappush(openpq, (h(s), s))
    gscore: Dict[Cell, float] = {s: 0.0}
    parent: Dict[Cell, Cell] = {}

    while openpq:
        _, cur = heapq.heappop(openpq)
        if cur == g:
            # reconstruct
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            path.reverse()
            return path

        for nb in grid.neighbors4(cur):
            if not grid.is_free(nb, occ_thr):
                continue
            tentative = gscore[cur] + 1.0
            if nb not in gscore or tentative < gscore[nb]:
                gscore[nb] = tentative
                parent[nb] = cur
                heapq.heappush(openpq, (tentative + h(nb), nb))
    return None

def detect_frontiers(grid: OccupancyGrid, free_thr: float = 0.35) -> List[Cell]:
    """
    A frontier cell: free (p_occ < free_thr) and has a 4-neighbor unknown-ish (close to 0.5).
    """
    frontiers = []
    for y in range(grid.h):
        for x in range(grid.w):
            c = (x, y)
            p = grid.get(c)
            if p >= free_thr:
                continue
            # unknown neighbor?
            for nb in grid.neighbors4(c):
                pnb = grid.get(nb)
                if abs(pnb - 0.5) < 0.15:
                    frontiers.append(c)
                    break
    return frontiers

def expected_info_gain(grid: OccupancyGrid, pose: Cell, sensor: SensorParams) -> float:
    """
    Approximate IG as entropy reduction over cells in sensor range, assuming observation makes
    p_occ closer to {p_occ_obs or p_free_obs}. In a real system, this would integrate
    measurement model over rays (LiDAR) or pixels (vision).
    """
    x0, y0 = pose
    ig = 0.0
    for dy in range(-sensor.r, sensor.r+1):
        for dx in range(-sensor.r, sensor.r+1):
            c = (x0+dx, y0+dy)
            if not grid.inb(c):
                continue
            p0 = grid.get(c)
            H0 = entropy_bern(p0)
            # expected posterior entropy (simple symmetric mixture proxy)
            # posterior becomes more "certain" than prior
            p_post1 = clamp(sensor.p_occ_obs, 0.0, 1.0)
            p_post0 = clamp(sensor.p_free_obs, 0.0, 1.0)
            H_post = 0.5 * entropy_bern(p_post1) + 0.5 * entropy_bern(p_post0)
            # only count if cell is not already certain
            w_unc = clamp(1.0 - abs(p0 - 0.5) * 2.0, 0.0, 1.0)
            ig += w_unc * max(0.0, H0 - H_post)
    return ig

def apply_observation(grid: OccupancyGrid, truth: OccupancyGrid, pose: Cell, sensor: SensorParams) -> None:
    """Simulate observation by revealing truth in sensor neighborhood with noisy posterior."""
    x0, y0 = pose
    for dy in range(-sensor.r, sensor.r+1):
        for dx in range(-sensor.r, sensor.r+1):
            c = (x0+dx, y0+dy)
            if not grid.inb(c):
                continue
            # truth is binary obstacle vs free by thresholding its probability
            is_occ = truth.is_occupied(c, thr=0.5)
            grid.set(c, sensor.p_occ_obs if is_occ else sensor.p_free_obs)

def pick_goal_budgeted(
    grid: OccupancyGrid,
    frontiers: List[Cell],
    cur: Cell,
    home: Cell,
    budget: Budget,
    rp: RobotParams,
    sensor: SensorParams,
    lam: float = 0.25,
    occ_thr: float = 0.65,
    max_candidates: int = 80
) -> Optional[Tuple[Cell, List[Cell], float]]:
    """
    Choose frontier maximizing U = IG(goal) - lam * travel_cost
    subject to energy/time feasibility INCLUDING return-to-home.
    """
    if not frontiers:
        return None

    # subsample for speed
    if len(frontiers) > max_candidates:
        frontiers = random.sample(frontiers, max_candidates)

    best = None
    bestU = -1e18
    best_path = None

    for g in frontiers:
        path = astar(grid, cur, g, occ_thr=occ_thr)
        if path is None:
            continue
        # cost to reach g
        steps = max(0, len(path) - 1)
        E_go = steps * (rp.E_step + rp.E_base)
        T_go = steps * rp.T_step

        # return cost (shortest path from g to home)
        path_back = astar(grid, g, home, occ_thr=occ_thr)
        if path_back is None:
            # if we cannot guarantee return on current map, treat as infeasible
            continue
        steps_back = max(0, len(path_back) - 1)
        E_back = steps_back * (rp.E_step + rp.E_base)
        T_back = steps_back * rp.T_step

        # feasibility with reserve
        if budget.E - (E_go + E_back) < rp.E_reserve:
            continue
        if budget.T - (T_go + T_back) < 0:
            continue

        ig = expected_info_gain(grid, g, sensor)
        U = ig - lam * (steps)  # travel cost proxy
        if U > bestU:
            bestU = U
            best = g
            best_path = path

    if best is None:
        return None
    return best, best_path, bestU

def run_demo():
    # grid and truth
    W, H = 35, 25
    belief = OccupancyGrid(W, H)
    truth = OccupancyGrid(W, H)

    # create synthetic obstacles in truth (maze-ish)
    for y in range(H):
        for x in range(W):
            truth.set((x,y), 0.05)  # mostly free

    # borders
    for x in range(W):
        truth.set((x,0), 0.95); truth.set((x,H-1), 0.95)
    for y in range(H):
        truth.set((0,y), 0.95); truth.set((W-1,y), 0.95)

    # random rectangles
    rng = random.Random(3)
    for _ in range(8):
        x0 = rng.randint(3, W-10)
        y0 = rng.randint(3, H-8)
        ww = rng.randint(3, 7)
        hh = rng.randint(2, 5)
        for y in range(y0, min(H-1, y0+hh)):
            for x in range(x0, min(W-1, x0+ww)):
                truth.set((x,y), 0.95)

    # start/home
    home = (2, 2)
    cur = home

    rp = RobotParams(E_step=1.2, T_step=1.0, E_base=0.10, E_reserve=15.0)
    sensor = SensorParams(r=3)

    # budgets
    budget = Budget(E=160.0, T=140.0)

    # initial observation
    apply_observation(belief, truth, cur, sensor)

    explored_steps = 0
    goals_reached = 0

    while True:
        frontiers = detect_frontiers(belief)
        if not frontiers:
            print("No frontiers left. Exploration complete.")
            break

        choice = pick_goal_budgeted(
            belief, frontiers, cur, home, budget, rp, sensor,
            lam=0.35, occ_thr=0.65
        )
        if choice is None:
            print("No feasible frontier under remaining budget. Returning home.")
            break

        goal, path, U = choice
        # execute path step-by-step, update budgets and sensing
        for step in path[1:]:
            # consume
            budget.E -= (rp.E_step + rp.E_base)
            budget.T -= rp.T_step
            explored_steps += 1
            cur = step
            apply_observation(belief, truth, cur, sensor)

            if budget.E < rp.E_reserve or budget.T <= 0:
                print("Budget exhausted mid-path. Returning home (if possible).")
                break
        goals_reached += 1

        # stop if too low to continue safely
        if budget.E < rp.E_reserve or budget.T <= 0:
            break

        if goals_reached >= 40:
            print("Stop: reached demo goal limit.")
            break

    # return home (best effort)
    back = astar(belief, cur, home, occ_thr=0.65)
    if back is not None:
        for step in back[1:]:
            budget.E -= (rp.E_step + rp.E_base)
            budget.T -= rp.T_step
            cur = step

    print(f"Final pose: {cur}, steps: {explored_steps}, goals: {goals_reached}")
    print(f"Remaining budget: E={budget.E:.2f}, T={budget.T:.2f}")

if __name__ == "__main__":
    run_demo()
