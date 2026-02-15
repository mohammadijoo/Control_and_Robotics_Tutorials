import control as ct

num = [1.0]
den = [1.0, 3.0, 2.0]
G = ct.TransferFunction(num, den)
print("Poles from python-control:", ct.pole(G))
