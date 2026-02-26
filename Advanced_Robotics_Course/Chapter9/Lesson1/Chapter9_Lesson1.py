from dataclasses import dataclass, field
from typing import FrozenSet, Tuple, List, Callable, Optional
import math

# ----------------------------------------------------------------------
# Symbolic planning structures
# ----------------------------------------------------------------------

Fluent = str
State = FrozenSet[Fluent]

@dataclass(frozen=True)
class Action:
    name: str
    pre: FrozenSet[Fluent]
    add: FrozenSet[Fluent]
    delete: FrozenSet[Fluent]

    def applicable(self, s: State) -> bool:
        return self.pre.issubset(s)

    def apply(self, s: State) -> State:
        assert self.applicable(s)
        return (s - self.delete) | self.add

# Example domain: pick-and-place of one object from table to shelf
FLUENTS = {
    "HandEmpty",
    "On(Box,Table)",
    "On(Box,Shelf)",
    "Holding(Box)",
}

ACTIONS = [
    Action(
        name="PickFromTable",
        pre=frozenset({"HandEmpty", "On(Box,Table)"}),
        add=frozenset({"Holding(Box)"}),
        delete=frozenset({"HandEmpty", "On(Box,Table)"}),
    ),
    Action(
        name="PlaceOnShelf",
        pre=frozenset({"Holding(Box)"}),
        add=frozenset({"On(Box,Shelf)", "HandEmpty"}),
        delete=frozenset({"Holding(Box)"}),
    ),
]

INIT: State = frozenset({"HandEmpty", "On(Box,Table)"})
GOAL: State = frozenset({"On(Box,Shelf)"})

# ----------------------------------------------------------------------
# Geometric layer: 2D point robot with circular obstacles
# ----------------------------------------------------------------------

Config = Tuple[float, float]

OBSTACLES = [
    ((0.5, 0.5), 0.2),  # center, radius
]

def in_collision(q: Config) -> bool:
    x, y = q
    for (cx, cy), r in OBSTACLES:
        if math.hypot(x - cx, y - cy) <= r:
            return True
    return False

def straight_line_collision_free(q_start: Config, q_goal: Config, steps: int = 50) -> bool:
    for i in range(steps + 1):
        alpha = i / steps
        x = (1 - alpha) * q_start[0] + alpha * q_goal[0]
        y = (1 - alpha) * q_start[1] + alpha * q_goal[1]
        if in_collision((x, y)):
            return False
    return True

# ----------------------------------------------------------------------
# Refinement map R(s): here we hard-code representative configurations
# ----------------------------------------------------------------------

REFINEMENT = {
    # Robot at table, box on table
    frozenset({"HandEmpty", "On(Box,Table)"}): (0.1, 0.1),
    # Robot grasps box near table
    frozenset({"Holding(Box)"}): (0.2, 0.2),
    # Robot at shelf, box on shelf, hand empty
    frozenset({"HandEmpty", "On(Box,Shelf)"}): (0.9, 0.9),
}

def refine_state(s: State) -> Optional[Config]:
    """
    Very coarse refinement: pick a nominal configuration for state s,
    or return None if unknown.
    """
    # Use subset matching to deal with extra fluents like HandEmpty
    for key_state, q in REFINEMENT.items():
        if key_state.issubset(s):
            return q
    return None

def geom_feasible_transition(s: State, a: Action, s_next: State) -> bool:
    """
    Check if symbolic transition s -a-> s_next has a straight-line
    geometric realization between representative configurations.
    """
    q = refine_state(s)
    q_next = refine_state(s_next)
    if q is None or q_next is None:
        return False
    if in_collision(q) or in_collision(q_next):
        return False
    return straight_line_collision_free(q, q_next)

# ----------------------------------------------------------------------
# Integrated search: BFS over symbolic states with geometric pruning
# ----------------------------------------------------------------------

@dataclass
class Node:
    state: State
    plan: List[Action] = field(default_factory=list)

def tamp_bfs(init: State, goal: State, actions: List[Action]) -> Optional[Node]:
    from collections import deque
    frontier = deque([Node(state=init, plan=[])])
    visited = {init}
    while frontier:
        node = frontier.popleft()
        if goal.issubset(node.state):
            return node
        for a in actions:
            if not a.applicable(node.state):
                continue
            s_next = a.apply(node.state)
            if s_next in visited:
                continue
            # Geometric feasibility pruning
            if not geom_feasible_transition(node.state, a, s_next):
                continue
            visited.add(s_next)
            frontier.append(Node(state=s_next, plan=node.plan + [a]))
    return None

if __name__ == "__main__":
    sol = tamp_bfs(INIT, GOAL, ACTIONS)
    if sol is None:
        print("No TAMP solution found.")
    else:
        print("Symbolic plan:")
        for a in sol.plan:
            print(" -", a.name)
        print("Final state:", sol.state)
      
