from collections import defaultdict, deque
from typing import Dict, Set, Tuple, List

State = Tuple[int, int]  # grid coordinates (i, j)

class TransitionSystem:
    def __init__(self):
        self.states: Set[State] = set()
        self.initial: State = None
        self.actions = ["up", "down", "left", "right"]
        self.transitions: Dict[Tuple[State, str], Set[State]] = defaultdict(set)
        self.ap: Set[str] = {"goal", "unsafe"}
        self.label: Dict[State, Set[str]] = dict()

    def add_state(self, s: State, is_initial: bool = False,
                  props: Set[str] = None):
        self.states.add(s)
        if is_initial:
            self.initial = s
        self.label[s] = set(props) if props is not None else set()

    def add_transition(self, s: State, a: str, s_next: State):
        self.transitions[(s, a)].add(s_next)

class BuchiAutomaton:
    def __init__(self):
        # For phi = F goal
        self.states: Set[str] = {"q0", "q1"}
        self.initial: str = "q0"
        self.accepting: Set[str] = {"q1"}

    def delta(self, q: str, sigma: Set[str]) -> str:
        has_goal = "goal" in sigma
        if q == "q0":
            return "q1" if has_goal else "q0"
        else:
            # q1 is absorbing
            return "q1"

def build_grid_ts(width: int, height: int,
                  obstacles: Set[State],
                  goal: State,
                  start: State) -> TransitionSystem:
    ts = TransitionSystem()
    for i in range(height):
        for j in range(width):
            s = (i, j)
            if s in obstacles:
                continue
            props = set()
            if s == goal:
                props.add("goal")
            ts.add_state(s, is_initial=(s == start), props=props)

    # 4-neighborhood
    dirs = {"up": (-1, 0), "down": (1, 0),
            "left": (0, -1), "right": (0, 1)}
    for s in ts.states:
        i, j = s
        for a, (di, dj) in dirs.items():
            s_next = (i + di, j + dj)
            if s_next in ts.states:
                ts.add_transition(s, a, s_next)
    return ts

def product_initial(ts: TransitionSystem, ba: BuchiAutomaton):
    return (ts.initial, ba.initial)

def product_successors(ts: TransitionSystem, ba: BuchiAutomaton,
                       s_prod: Tuple[State, str]) -> List[Tuple[Tuple[State, str], str]]:
    (s, q) = s_prod
    result = []
    for a in ts.actions:
        for s_next in ts.transitions.get((s, a), []):
            sigma = ts.label[s]
            q_next = ba.delta(q, sigma)
            result.append(((s_next, q_next), a))
    return result

def find_accepting_lasso(ts: TransitionSystem, ba: BuchiAutomaton):
    start = product_initial(ts, ba)
    # BFS for reachable states and predecessor map
    queue = deque([start])
    pred: Dict[Tuple[State, str], Tuple[Tuple[State, str], str]] = {}
    visited = {start}
    accepting_states = set()

    while queue:
        cur = queue.popleft()
        (s, q) = cur
        if q in ba.accepting:
            accepting_states.add(cur)
        for nxt, act in product_successors(ts, ba, cur):
            if nxt not in visited:
                visited.add(nxt)
                pred[nxt] = (cur, act)
                queue.append(nxt)

    if not accepting_states:
        return None  # no plan

    # For simplicity, pick one accepting state and search for a cycle to itself
    acc = next(iter(accepting_states))

    # DFS within visited states to find a cycle starting and ending at acc
    def dfs_cycle(cur, target, visited_local, path):
        if cur == target and path:
            return path
        visited_local.add(cur)
        for nxt, act in product_successors(ts, ba, cur):
            if nxt not in visited or nxt not in visited_local:
                res = dfs_cycle(nxt, target, visited_local, path + [(nxt, act)])
                if res is not None:
                    return res
        return None

    # Recover prefix to acc
    prefix_states_actions = []
    cur = acc
    while cur != start:
        prev, act = pred[cur]
        prefix_states_actions.append((cur, act))
        cur = prev
    prefix_states_actions.reverse()

    cycle = dfs_cycle(acc, acc, set(), [])
    if cycle is None:
        return None  # no lasso found

    return start, prefix_states_actions, cycle

if __name__ == "__main__":
    width, height = 4, 3
    obstacles = {(1, 1)}
    goal = (0, 3)
    start = (2, 0)
    ts = build_grid_ts(width, height, obstacles, goal, start)
    ba = BuchiAutomaton()
    lasso = find_accepting_lasso(ts, ba)
    if lasso is None:
        print("No plan satisfying F(goal) found.")
    else:
        start, prefix, cycle = lasso
        print("Prefix:")
        print(prefix)
        print("Cycle:")
        print(cycle)
      
