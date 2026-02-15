import numpy as np

# Task specs: (C_i FLOPs, S_i bits, S_r_i bits, deadline D_i s)
tasks = [
    (8e8, 5e6, 2e5, 0.20),   # heavy compute, moderate data
    (1e7, 2e6, 1e5, 0.05),   # light compute, tight deadline
    (5e8, 1e7, 5e5, 0.50)    # heavy compute, large input
]

f_on  = 2e9    # onboard FLOPs/s
f_off = 2e11   # offboard FLOPs/s (edge GPU)

B_u, B_d = 50e6, 80e6  # uplink/downlink bits/s
T_p = 0.008           # propagation/stack delay (s)

P_on = 12.0           # onboard compute power (W)
P_tx, P_rx = 2.5, 1.8 # radio powers (W)
P_idle = 6.0          # idle robot power while waiting (W)

def times_and_energy(C, S, Sr):
    T_on = C / f_on
    T_off = S/B_u + 2*T_p + C/f_off + Sr/B_d

    E_on = P_on * T_on
    E_off = P_tx*(S/B_u) + P_rx*(Sr/B_d) + P_idle*(2*T_p + C/f_off)
    return T_on, T_off, E_on, E_off

results = []
for i, (C,S,Sr,D) in enumerate(tasks):
    T_on, T_off, E_on, E_off = times_and_energy(C,S,Sr)
    results.append((i, T_on, T_off, E_on, E_off, D))

# Greedy by time savings while meeting deadlines
results.sort(key=lambda r: r[1]-r[2], reverse=True)

placement = {}
for i, T_on, T_off, E_on, E_off, D in results:
    if T_off <= D:
        placement[i] = "offboard" if T_off < T_on else "onboard"
    else:
        placement[i] = "onboard"  # must keep onboard to meet deadline

for r in results:
    i, T_on, T_off, E_on, E_off, D = r
    print(f"Task {i}: Ton={T_on:.3f}s, Toff={T_off:.3f}s, "
          f"Eon={E_on:.2f}J, Eoff={E_off:.2f}J, D={D:.2f}s -> {placement[i]}")
