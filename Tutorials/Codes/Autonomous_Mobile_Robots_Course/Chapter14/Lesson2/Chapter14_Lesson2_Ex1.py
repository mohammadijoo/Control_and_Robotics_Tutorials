"""
Chapter14_Lesson2_Ex1.py
Exercise: empirically verify monotonicity of the inflation cost and visualize cost(d).
"""

import math

def inflation_cost(d: float, r_inscribed: float, r_inflation: float, k: float,
                   inscribed_cost: int = 253) -> int:
    if d <= 0.0:
        return 254
    if d <= r_inscribed:
        return inscribed_cost
    if d > r_inflation:
        return 0
    c = (inscribed_cost - 1) * math.exp(-k * (d - r_inscribed)) + 1.0
    return int(max(1, min(inscribed_cost - 1, round(c))))

def main():
    r_ins, r_inf, k = 0.25, 0.60, 6.0
    ds = [i * 0.01 for i in range(0, 121)]
    cs = [inflation_cost(d, r_ins, r_inf, k) for d in ds]

    # check monotone non-increasing for d >= r_ins
    ok = True
    for i in range(1, len(ds)):
        if ds[i] >= r_ins and cs[i] > cs[i-1]:
            ok = False
            print("Violation at d=", ds[i-1], "->", ds[i], "cost", cs[i-1], "->", cs[i])
            break
    print("Monotone non-increasing beyond inscribed radius:", ok)

    # print a small table
    for d in [0.0, 0.1, 0.25, 0.30, 0.40, 0.60, 0.80]:
        print(f"d={d:0.2f} m -> cost={inflation_cost(d, r_ins, r_inf, k)}")

if __name__ == "__main__":
    main()
