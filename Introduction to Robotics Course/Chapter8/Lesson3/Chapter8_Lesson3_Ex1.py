import numpy as np

def edge_map_finite_difference(I, thresh=30.0):
    # I: 2D float image
    Kx = np.array([[-0.5, 0.0, 0.5]])
    Ky = Kx.T
    # convolution (naive)
    Ix = np.zeros_like(I)
    Iy = np.zeros_like(I)
    for y in range(1, I.shape[0]-1):
        for x in range(1, I.shape[1]-1):
            patch = I[y-1:y+2, x-1:x+2]
            Ix[y,x] = np.sum(patch * Kx)
            Iy[y,x] = np.sum(patch * Ky)
    G = np.sqrt(Ix*Ix + Iy*Iy)
    return (G > thresh).astype(np.uint8)

# example:
# edges = edge_map_finite_difference(img.astype(float))
