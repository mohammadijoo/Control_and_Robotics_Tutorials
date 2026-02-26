# Requires a CUDA-capable GPU and cupy installed
import cupy as cp

A = cp.array([[1.0, 0.01],
              [0.0, 1.0]])
B = cp.array([[0.0],
              [0.01]])
K = cp.array([[2.0, 0.5]])

# Batch of N states (2 x N)
N = 100000
x = cp.random.randn(2, N) * 0.1

for k in range(1000):
    u = -(K @ x)              # 1 x N
    x = A @ x + B @ u         # 2 x N
cp.cuda.Stream.null.synchronize()
print(cp.mean(x, axis=1))
