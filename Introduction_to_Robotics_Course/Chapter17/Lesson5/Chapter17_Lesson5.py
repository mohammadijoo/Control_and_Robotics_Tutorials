import numpy as np

def compute_ndvi(nir: np.ndarray, red: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """
    Compute NDVI = (nir - red) / (nir + red), with small epsilon to avoid division by zero.
    Inputs:
        nir, red: 2D arrays of the same shape containing reflectance values.
    """
    denom = nir + red + eps
    ndvi = (nir - red) / denom
    # Clip to physical range [-1, 1]
    ndvi = np.clip(ndvi, -1.0, 1.0)
    return ndvi

def stressed_mask(ndvi: np.ndarray, threshold: float) -> np.ndarray:
    """
    Return boolean mask where NDVI is below a stress threshold.
    """
    return ndvi < threshold

def generate_strip_waypoints(Lx: float, Ly: float, s: float) -> list:
    """
    Generate a list of waypoints for a simple back-and-forth pattern over [0,Lx] x [0,Ly].
    The robot flies along x, stepping in y with spacing s.
    """
    N_pass = int(np.ceil(Ly / s))
    waypoints = []
    y = 0.0
    for k in range(N_pass):
        if k % 2 == 0:
            # Even index: go from (0, y) to (Lx, y)
            waypoints.append((0.0, y))
            waypoints.append((Lx, y))
        else:
            # Odd index: come back from (Lx, y) to (0, y)
            waypoints.append((Lx, y))
            waypoints.append((0.0, y))
        y = min(Ly, y + s)
    return waypoints

# Example usage (assuming we have loaded NIR and RED arrays):
nir = np.random.rand(512, 512)
red = np.random.rand(512, 512)

ndvi = compute_ndvi(nir, red)
mask = stressed_mask(ndvi, threshold=0.3)

Lx, Ly = 100.0, 50.0   # meters
s = 5.0                # footprint spacing
wps = generate_strip_waypoints(Lx, Ly, s)

print("Number of waypoints:", len(wps))
# In a full robot system, 'wps' would be sent to the guidance module,
# while 'mask' would be used to trigger localized actions (e.g., spraying).
      
