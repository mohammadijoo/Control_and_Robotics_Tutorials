from dataclasses import dataclass
from collections import deque, defaultdict
from typing import Dict, List, Set

@dataclass
class Task:
    name: str
    duration: float               # hours or days
    predecessors: List[str]       # task names that must finish before this starts

def topological_order(tasks: Dict[str, Task]) -> List[str]:
    """Kahn's algorithm for DAG topological ordering."""
    successors = defaultdict(list)
    indegree = defaultdict(int)

    for t in tasks.values():
        indegree[t.name] = indegree[t.name]  # ensure key exists
        for p in t.predecessors:
            successors[p].append(t.name)
            indegree[t.name] += 1

    q = deque([name for name, deg in indegree.items() if deg == 0])
    order = []
    while q:
        v = q.popleft()
        order.append(v)
        for w in successors[v]:
            indegree[w] -= 1
            if indegree[w] == 0:
                q.append(w)
    if len(order) != len(tasks):
        raise ValueError("Graph has a cycle or missing tasks")
    return order

def earliest_times(tasks: Dict[str, Task]) -> Dict[str, float]:
    """Compute earliest finish time for each task."""
    order = topological_order(tasks)
    finish: Dict[str, float] = {}
    start: Dict[str, float] = {}

    for name in order:
        t = tasks[name]
        if not t.predecessors:
            start[name] = 0.0
        else:
            start[name] = max(finish[p] for p in t.predecessors)
        finish[name] = start[name] + t.duration
    return finish

def milestone_time(tasks: Dict[str, Task], milestone_tasks: Set[str]) -> float:
    finish = earliest_times(tasks)
    return max(finish[name] for name in milestone_tasks)

# Example: hardware bring-up milestone
tasks = {
    "chassis": Task("chassis", 8.0, []),
    "motor_wiring": Task("motor_wiring", 6.0, ["chassis"]),
    "mcu_bringup": Task("mcu_bringup", 5.0, []),
    "low_voltage_test": Task("low_voltage_test", 4.0,
                             ["motor_wiring", "mcu_bringup"]),
}

M1 = {"chassis", "motor_wiring", "mcu_bringup", "low_voltage_test"}
T_M1 = milestone_time(tasks, M1)
print("Milestone M1 earliest completion time:", T_M1)
      
