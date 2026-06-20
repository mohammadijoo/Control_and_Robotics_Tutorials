"""
Exercise — Chapter 14 Lesson 1 (Nonlinearities)

A quick computational check of linearity properties for memoryless maps f(x).
We test additivity and homogeneity:
    f(x1 + x2) ?= f(x1) + f(x2)
    f(a x) ?= a f(x)
"""

from __future__ import annotations
import numpy as np


def check_linear(f, xs, a=1.7, tol=1e-10):
    x1, x2 = xs
    add = np.max(np.abs(f(x1 + x2) - (f(x1) + f(x2))))
    hom = np.max(np.abs(f(a * x1) - a * f(x1)))
    return add < tol and hom < tol, add, hom


def main():
    rng = np.random.default_rng(7)
    x1 = rng.normal(size=1000)
    x2 = rng.normal(size=1000)

    funcs = {
        "f(x)=x": lambda x: x,
        "f(x)=2x": lambda x: 2.0 * x,
        "f(x)=x+1": lambda x: x + 1.0,
        "f(x)=x^2": lambda x: x**2,
        "f(x)=tanh(x)": lambda x: np.tanh(x),
        "f(x)=sat(x,1)": lambda x: np.clip(x, -1.0, 1.0),
    }

    for name, f in funcs.items():
        ok, add_err, hom_err = check_linear(f, (x1, x2))
        print(f"{name:14s}  linear? {ok}  add_err={add_err:.3e}  hom_err={hom_err:.3e}")


if __name__ == "__main__":
    main()
