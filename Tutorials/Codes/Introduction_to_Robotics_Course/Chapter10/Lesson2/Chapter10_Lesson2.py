import math
from dataclasses import dataclass

@dataclass
class Task:
    C: float  # WCET
    T: float  # period
    D: float  # deadline (assume D=T by default)

def utilization(tasks):
    return sum(t.C / t.T for t in tasks)

def rm_bound(n):
    return n*(2**(1/n) - 1)

def rta_schedulable(tasks):
    # tasks sorted by increasing T (RM priorities)
    tasks = sorted(tasks, key=lambda x: x.T)
    for i, ti in enumerate(tasks):
        R = ti.C
        while True:
            interference = 0.0
            for tj in tasks[:i]:
                interference += math.ceil(R / tj.T) * tj.C
            R_next = ti.C + interference
            if R_next == R:
                break
            if R_next > ti.D:
                return False
            R = R_next
    return True

tasks = [
    Task(C=1.0, T=5.0, D=5.0),
    Task(C=1.5, T=8.0, D=8.0),
    Task(C=2.0, T=12.0, D=12.0)
]

U = utilization(tasks)
print("Utilization:", U)
print("RM sufficient bound:", rm_bound(len(tasks)))
print("RM schedulable by bound?", U <= rm_bound(len(tasks)))
print("RM schedulable by RTA?", rta_schedulable(tasks))

# Simple jitter simulation: intended T=10ms with bounded noise
import numpy as np
np.random.seed(0)
T = 0.010  # 10 ms
Jmax = 0.002
N = 1000
delta = np.random.uniform(-Jmax, Jmax, size=N)
t_k = np.arange(N)*T + delta
T_k = np.diff(t_k)

print("Observed sampling period mean:", T_k.mean())
print("Observed period std (jitter):", T_k.std())
