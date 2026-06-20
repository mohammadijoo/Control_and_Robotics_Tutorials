if control is not None:
    num = [5.0]
    den = [1.0, 1.1, 0.5]  # Example polynomial; adjust to match your plant
    L_sys = control.TransferFunction(num, den)
    control.nyquist(L_sys)  # built-in Nyquist plot
