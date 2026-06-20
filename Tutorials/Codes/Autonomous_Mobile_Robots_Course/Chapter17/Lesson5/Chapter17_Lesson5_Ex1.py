# Chapter17_Lesson5_Ex1.py
"""
Exercise 1 (Frontier clustering + goal selection sanity check)

Task:
1) Load a random belief map (synthetic).
2) Detect frontier cells.
3) Cluster them (4-neighbor).
4) Print cluster sizes and centroids.
5) Verify visually (optional matplotlib).

Expected learning:
- Correct implementation of frontier detection: unknown adjacent to known-free.
- Robust clustering that ignores tiny spurious groups.
"""
import numpy as np

def neighbors4(x, y):
    return [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]

def frontier_cells(class_map):
    h, w = class_map.shape
    fronts = []
    for y in range(1, h-1):
        for x in range(1, w-1):
            if class_map[y, x] != -1:
                continue
            if any(class_map[ny, nx] == 0 for (nx, ny) in neighbors4(x, y)):
                fronts.append((x, y))
    return fronts

def cluster_frontiers(fronts):
    s = set(fronts)
    visited = set()
    clusters = []
    for cell in fronts:
        if cell in visited: 
            continue
        q = [cell]
        visited.add(cell)
        c = []
        while q:
            x,y = q.pop()
            c.append((x,y))
            for nx,ny in neighbors4(x,y):
                if (nx,ny) in s and (nx,ny) not in visited:
                    visited.add((nx,ny))
                    q.append((nx,ny))
        clusters.append(c)
    return [c for c in clusters if len(c) >= 5]

def centroid(cluster):
    xs = [p[0] for p in cluster]
    ys = [p[1] for p in cluster]
    return int(round(sum(xs)/len(xs))), int(round(sum(ys)/len(ys)))

def main():
    np.random.seed(3)
    h,w = 30,40
    # synthetic class map: -1 unknown, 0 free, 1 occupied
    cls = -np.ones((h,w), dtype=np.int8)
    # make a free region + obstacle strip
    cls[10:20, 8:30] = 0
    cls[14:16, 15:25] = 1

    fronts = frontier_cells(cls)
    clusters = cluster_frontiers(fronts)
    print("frontier cells:", len(fronts))
    print("clusters:", len(clusters))
    for i,c in enumerate(sorted(clusters, key=len, reverse=True)[:10]):
        print(f"cluster {i}: size={len(c)} centroid={centroid(c)}")

    try:
        import matplotlib.pyplot as plt
        plt.figure()
        plt.title("Synthetic class map")
        plt.imshow(cls, origin="lower")
        xs = [p[0] for p in fronts]
        ys = [p[1] for p in fronts]
        plt.scatter(xs, ys, s=5)
        plt.show()
    except Exception as e:
        print("matplotlib skipped:", e)

if __name__ == "__main__":
    main()
