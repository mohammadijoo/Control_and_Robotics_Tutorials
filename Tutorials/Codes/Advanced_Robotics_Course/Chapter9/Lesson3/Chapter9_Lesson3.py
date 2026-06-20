from typing import Callable, Dict, List, Tuple, Any, Iterable
import random
import math

Fact = Tuple[str, Tuple[Any, ...]]  # e.g. ("at", ("block1", "table1"))
Plan = List[str]

class Stream:
    def __init__(self, name: str,
                 input_vars: Tuple[str, ...],
                 output_vars: Tuple[str, ...],
                 precond: Callable[[Dict[str, Any], List[Fact]], bool],
                 sampler: Callable[[Dict[str, Any]], Iterable[Dict[str, Any]]],
                 head_predicates: Callable[[Dict[str, Any]], List[Fact]]):
        self.name = name
        self.input_vars = input_vars
        self.output_vars = output_vars
        self.precond = precond
        self.sampler = sampler
        self.head_predicates = head_predicates

    def call(self, binding: Dict[str, Any], facts: List[Fact]) -> List[Fact]:
        """Return first successful sample and its derived facts, or empty list."""
        if not self.precond(binding, facts):
            return []
        for sample_binding in self.sampler(binding):
            facts_new = self.head_predicates(sample_binding)
            if facts_new:
                return facts_new
        return []

# Example: a grasp stream for a block
def grasp_precond(binding: Dict[str, Any], facts: List[Fact]) -> bool:
    block = binding["b"]
    return ("movable", (block,)) in facts

def grasp_sampler(binding: Dict[str, Any]):
    block = binding["b"]
    for _ in range(100):
        theta = random.uniform(0.0, 2.0 * math.pi)
        yield {"b": block, "g": ("top_grasp", theta)}

def grasp_head(binding: Dict[str, Any]) -> List[Fact]:
    b = binding["b"]
    g = binding["g"]
    return [("grasp", (b, g))]

grasp_stream = Stream(
    name="sample-grasp",
    input_vars=("b",),
    output_vars=("g",),
    precond=grasp_precond,
    sampler=grasp_sampler,
    head_predicates=grasp_head,
)

# Motion planning stream that calls a sampling-based planner (placeholder)
def motion_precond(binding: Dict[str, Any], facts: List[Fact]) -> bool:
    return True

def motion_sampler(binding: Dict[str, Any]):
    q_start = binding["q_start"]
    q_goal = binding["q_goal"]
    # Here we would call OMPL or MoveIt and yield a trajectory if found.
    # We simulate a success with some probability.
    if random.random() < 0.7:
        yield {"traj": [q_start, q_goal]}

def motion_head(binding: Dict[str, Any]) -> List[Fact]:
    return [("reachable", (binding["q_start"], binding["q_goal"]))]

motion_stream = Stream(
    name="sample-motion",
    input_vars=("q_start", "q_goal"),
    output_vars=("traj",),
    precond=motion_precond,
    sampler=motion_sampler,
    head_predicates=motion_head,
)

# A very small "focused" loop (symbolic planner is abstracted as a function)
def focused_tamp(initial_facts: List[Fact],
                 streams: List[Stream],
                 max_iters: int = 20):
    known_facts = list(initial_facts)
    for it in range(max_iters):
        plan_skeleton, stream_calls = symbolic_planner_stub(known_facts)
        if plan_skeleton is None:
            print("No plan skeleton; stopping.")
            return None

        print(f"Iteration {it}, trying plan skeleton:", plan_skeleton)

        success = True
        for stream, binding in stream_calls:
            new_facts = stream.call(binding, known_facts)
            if not new_facts:
                # record failure as a fact and break
                fail_fact = ("failed-stream", (stream.name, tuple(binding.items())))
                known_facts.append(fail_fact)
                success = False
                break
            else:
                for f in new_facts:
                    if f not in known_facts:
                        known_facts.append(f)

        if success:
            print("Realized TAMP plan!")
            return plan_skeleton
    return None

def symbolic_planner_stub(facts: List[Fact]):
    # In practice, call a PDDL planner (FastDownward, pyperplan, etc.)
    # Here we return a dummy plan and dummy stream calls.
    if random.random() < 0.3:
        return None, []
    plan = ["pick b1", "place b1"]
    stream_calls = [
        (grasp_stream, {"b": "b1"}),
        (motion_stream, {"q_start": (0.0, 0.0), "q_goal": (1.0, 0.0)}),
    ]
    return plan, stream_calls
      
