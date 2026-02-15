# Approximate the critical delay using phase margin
Td_critical = Td_from_pm

# First-order Pade approximation of e^{-s T}
num_pade, den_pade = ctl.pade(Td_critical, 1)
Delay_approx = ctl.TransferFunction(num_pade, den_pade)

L_delayed_approx = L * Delay_approx
T_cl = ctl.feedback(L_delayed_approx, 1)

poles = ctl.pole(T_cl)
print("Closed-loop poles with approximate critical delay:", poles)
