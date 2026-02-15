import numpy as np

np.random.seed(1)
N = 200
X = np.random.randn(N, 2)

w_true = np.array([1.2, -0.8])
b_true = -0.1
y = (1/(1+np.exp(-(X @ w_true + b_true))) > 0.5).astype(int)

w = np.zeros(2); b = 0.0
eta = 0.1

def sigmoid(z):
    return 1/(1+np.exp(-z))

for k in range(2000):
    p = sigmoid(X @ w + b)
    grad_w = (1/N) * X.T @ (p - y)
    grad_b = (1/N) * np.sum(p - y)
    w -= eta * grad_w
    b -= eta * grad_b

acc = np.mean((sigmoid(X @ w + b) > 0.5) == y)
print("learned w:", w, "learned b:", b)
print("training accuracy:", acc)
      