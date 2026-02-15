from __future__ import annotations
from dataclasses import dataclass
from typing import Set, List, Union, Dict

# --- Representation of a labeled transition system (finite depth) ---

State = str
AtomicProp = str

@dataclass
class TransitionSystem:
    states: Set[State]
    init: State
    edges: Dict[State, List[State]]
    labels: Dict[State, Set[AtomicProp]]

    def run(self, policy, horizon: int) -> List[State]:
        """
        Simulate one run of length 'horizon' under a simple policy.
        'policy' maps current state to a successor state.
        """
        path = [self.init]
        s = self.init
        for _ in range(horizon):
            s = policy(s, self.edges[s])
            path.append(s)
        return path

    def label_trace(self, path: List[State]) -> List[Set[AtomicProp]]:
        return [self.labels[s] for s in path]

# --- LTL formula AST ---

class Formula:
    pass

@dataclass
class AP(Formula):
    name: AtomicProp

@dataclass
class Not(Formula):
    sub: Formula

@dataclass
class And(Formula):
    left: Formula
    right: Formula

@dataclass
class Or(Formula):
    left: Formula
    right: Formula

@dataclass
class Next(Formula):
    sub: Formula

@dataclass
class Until(Formula):
    left: Formula
    right: Formula

@dataclass
class Globally(Formula):
    sub: Formula

@dataclass
class Eventually(Formula):
    sub: Formula

# --- LTL semantics over a finite trace (bounded check) ---

def holds(formula: Formula,
          trace: List[Set[AtomicProp]],
          i: int = 0) -> bool:
    """
    Bounded semantics on a finite trace l_0,...,l_{N-1}.
    For temporal operators that require 'future' steps, we truncate at the end.
    """
    N = len(trace)

    if isinstance(formula, AP):
        return formula.name in trace[i]
    if isinstance(formula, Not):
        return not holds(formula.sub, trace, i)
    if isinstance(formula, And):
        return holds(formula.left, trace, i) and holds(formula.right, trace, i)
    if isinstance(formula, Or):
        return holds(formula.left, trace, i) or holds(formula.right, trace, i)
    if isinstance(formula, Next):
        if i + 1 >= N:
            return False
        return holds(formula.sub, trace, i + 1)
    if isinstance(formula, Globally):
        return all(holds(formula.sub, trace, j) for j in range(i, N))
    if isinstance(formula, Eventually):
        return any(holds(formula.sub, trace, j) for j in range(i, N))
    if isinstance(formula, Until):
        # phi U psi: exists j>=i with psi at j and phi at all k in [i,j)
        for j in range(i, N):
            if holds(formula.right, trace, j):
                if all(holds(formula.left, trace, k) for k in range(i, j)):
                    return True
        return False
    raise TypeError(f"Unknown formula type: {type(formula)}")

# --- Example: small navigation abstraction ---

# States: s0 (start), s1 (corridor), s2 (goal), sx (collision state)
ts = TransitionSystem(
    states={"s0", "s1", "s2", "sx"},
    init="s0",
    edges={
        "s0": ["s1", "sx"],
        "s1": ["s2", "sx"],
        "s2": ["s2"],
        "sx": ["sx"],
    },
    labels={
        "s0": set(),
        "s1": set(),
        "s2": {"goal"},
        "sx": {"collision"},
    },
)

def safe_policy(s: State, succ: List[State]) -> State:
    # Avoid successor states labeled with "collision" if possible
    for t in succ:
        if "collision" not in ts.labels[t]:
            return t
    return succ[0]

path = ts.run(safe_policy, horizon=5)
trace = ts.label_trace(path)

phi_safety = Globally(Not(AP("collision")))
phi_reach_goal = Eventually(AP("goal"))
phi_task = And(phi_safety, phi_reach_goal)

print("Path:", path)
print("Safety holds?", holds(phi_safety, trace))
print("Reach goal holds?", holds(phi_reach_goal, trace))
print("Combined task holds?", holds(phi_task, trace))
      
