
import numpy as np
import torch
from torch import nn

# True and nominal parameters for a 1-DOF rotary arm
I_true = 1.2
b_true = 0.3
g_true = 9.81

I_nom = 1.0
b_nom = 0.4
g_nom = 9.0

dt = 0.002

def true_ddq(q, dq, tau):
    # True acceleration
    return (tau - b_true * dq - g_true * np.sin(q)) / I_true

def nominal_tau(q, dq, ddq_des):
    # Nominal inverse dynamics: tau_nom = I_nom * ddq_des + b_nom * dq + g_nom * sin(q)
    return I_nom * ddq_des + b_nom * dq + g_nom * np.sin(q)

class ResidualNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(4, 64),
            nn.Tanh(),
            nn.Linear(64, 1)
        )

    def forward(self, x):
        return self.net(x)

# Data collection with a simple PD baseline
def collect_data(num_steps=20000):
    q = 0.0
    dq = 0.0
    qd = 0.5
    dqd = 0.0

    Kp = 20.0
    Kd = 4.0

    inputs = []
    targets = []

    for k in range(num_steps):
        e = qd - q
        de = dqd - dq
        ddq_des = Kp * e + Kd * de
        tau_nom = nominal_tau(q, dq, ddq_des)

        # Assume commanded torque equals tau_nom during data collection
        ddq = true_ddq(q, dq, tau_nom)

        # Features: [q, dq, ddq_des, e]
        z = np.array([q, dq, ddq_des, e], dtype=np.float32)
        # Residual target: tau_true - tau_nom
        tau_true = tau_nom  # because we use tau_nom in simulation
        # Here we artificially introduce model error as an unknown disturbance
        disturbance = 0.2 * np.tanh(dq) + 0.1 * np.sin(3.0 * q)
        tau_true = tau_true + disturbance

        y = tau_true - tau_nom

        inputs.append(z)
        targets.append(np.array([y], dtype=np.float32))

        # Simulate forward with the true dynamics including the disturbance
        ddq_true = (tau_true - b_true * dq - g_true * np.sin(q)) / I_true
        dq = dq + dt * ddq_true
        q = q + dt * dq

    X = torch.tensor(np.stack(inputs))
    Y = torch.tensor(np.stack(targets))
    return X, Y

net = ResidualNet()
optimizer = torch.optim.Adam(net.parameters(), lr=1e-3)
loss_fn = nn.MSELoss()

X, Y = collect_data()

for epoch in range(20):
    idx = torch.randint(0, X.shape[0], (1024,))
    xb = X[idx]
    yb = Y[idx]

    pred = net(xb)
    loss = loss_fn(pred, yb)

    optimizer.zero_grad()
    loss.backward()
    optimizer.step()

# Use in control loop
def control_step(q, dq, qd, dqd):
    Kp = 20.0
    Kd = 4.0
    e = qd - q
    de = dqd - dq
    ddq_des = Kp * e + Kd * de
    tau_nom = nominal_tau(q, dq, ddq_des)
    z = torch.tensor([[q, dq, ddq_des, e]], dtype=torch.float32)
    tau_res = float(net(z).detach().numpy()[0, 0])
    tau_cmd = tau_nom + tau_res
    return tau_cmd
