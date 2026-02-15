import numpy as np

# Synthetic feature generator for (x,a) pairs.
# In practice, phi(x,a) would use open3d, ROS, or MoveIt! to compute
# geometric features in the gripper frame.
def phi(x, a):
    """
    x: state features, shape (n_state,)
    a: action parameters, shape (n_action,)
    returns: feature vector, shape (d,)
    """
    # Concatenate and add simple interactions as a placeholder
    xa = np.concatenate([x, a])
    interactions = np.outer(x, a).flatten()
    return np.concatenate([xa, interactions])

def sigmoid(z):
    return 1.0 / (1.0 + np.exp(-z))

class LogisticAffordance:
    def __init__(self, d, lr=1e-2, reg=1e-3):
        self.theta = np.zeros(d)
        self.lr = lr
        self.reg = reg

    def predict_proba(self, phi_batch):
        z = phi_batch @ self.theta
        return sigmoid(z)

    def loss_and_grad(self, phi_batch, y_batch):
        p = self.predict_proba(phi_batch)
        # Binary cross-entropy with L2 regularization
        eps = 1e-8
        loss = -np.mean(y_batch * np.log(p + eps)
                        + (1.0 - y_batch) * np.log(1.0 - p + eps))
        loss += 0.5 * self.reg * np.dot(self.theta, self.theta)

        grad = -(phi_batch.T @ (y_batch - p)) / y_batch.shape[0]
        grad += self.reg * self.theta
        return loss, grad

    def step(self, phi_batch, y_batch):
        loss, grad = self.loss_and_grad(phi_batch, y_batch)
        self.theta -= self.lr * grad
        return loss

# Example usage with synthetic data
rng = np.random.default_rng(0)
n_state = 3
n_action = 3
d = (n_state + n_action) + n_state * n_action

model = LogisticAffordance(d=d, lr=5e-2, reg=1e-3)

N = 2000
X = rng.normal(size=(N, n_state))
A = rng.normal(size=(N, n_action))

# Ground-truth theta_star (unknown in practice)
theta_star = rng.normal(size=d)
Phi = np.stack([phi(X[i], A[i]) for i in range(N)], axis=0)
logits = Phi @ theta_star
p_true = sigmoid(logits)
Y = rng.binomial(1, p_true)  # Bernoulli labels (success/failure)

for epoch in range(50):
    # Mini-batch SGD
    idx = rng.permutation(N)
    Phi_shuf = Phi[idx]
    Y_shuf = Y[idx]

    for start in range(0, N, 128):
        end = min(start + 128, N)
        loss = model.step(Phi_shuf[start:end], Y_shuf[start:end])

    if (epoch % 10) == 0:
        print(f"Epoch {epoch}, loss {loss:.4f}")

# Now model.predict_proba can be used as an affordance score.
      
