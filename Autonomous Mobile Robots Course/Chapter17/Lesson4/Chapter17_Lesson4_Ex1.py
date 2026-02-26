# Chapter17_Lesson4_Ex1.py
# Exercise: Greedy knapsack selection for monotone submodular set function
# We illustrate the (1-1/e) style idea using a toy coverage objective.

from __future__ import annotations
from typing import List, Set, Tuple
import math

def greedy_submodular_knapsack(
    items: List[Tuple[str, Set[int], float]],
    B: float
) -> List[str]:
    """
    items: list of (name, covered_elements, cost)
    objective: f(S) = |union covered|
    choose S with sum(cost) <= B maximizing f(S)
    greedy by marginal gain per cost.
    """
    chosen = []
    covered: Set[int] = set()
    remaining = items[:]
    budget = B

    while True:
        best = None
        best_ratio = -1.0
        for name, elems, cost in remaining:
            if cost > budget:
                continue
            marg = len((covered | elems)) - len(covered)
            ratio = marg / cost if cost > 0 else float("inf")
            if ratio > best_ratio and marg > 0:
                best_ratio = ratio
                best = (name, elems, cost)

        if best is None:
            break

        name, elems, cost = best
        chosen.append(name)
        covered |= elems
        budget -= cost
        remaining = [it for it in remaining if it[0] != name]

    return chosen

if __name__ == "__main__":
    # toy environment elements 0..19 to "cover"
    items = [
        ("A", {0,1,2,3,4,5}, 3.0),
        ("B", {4,5,6,7,8}, 2.0),
        ("C", {9,10,11,12}, 2.5),
        ("D", {12,13,14,15,16}, 3.0),
        ("E", {16,17,18,19}, 1.8),
        ("F", {2,3,8,9,17}, 2.2),
    ]
    B = 7.0
    S = greedy_submodular_knapsack(items, B)
    print("Budget:", B)
    print("Chosen:", S)
