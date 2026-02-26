/*
Autonomous Mobile Robots — Chapter 17, Lesson 1: Frontier-Based Exploration
File: Chapter17_Lesson1.java

Java reference implementation of frontier detection and clustering on a grid.
Grid encoding:
  -1 = unknown, 0 = free, 100 = occupied

Note: In ROS ecosystems, Java integration can be done via rosjava or custom bridges.
For teaching, we keep this self-contained.
*/

import java.util.*;

public class Chapter17_Lesson1 {

    static class Coord {
        final int r, c;
        Coord(int r, int c) { this.r = r; this.c = c; }
        @Override public boolean equals(Object o) {
            if (!(o instanceof Coord)) return false;
            Coord other = (Coord)o;
            return r == other.r && c == other.c;
        }
        @Override public int hashCode() { return Objects.hash(r, c); }
    }

    static boolean inBounds(int H, int W, int r, int c) {
        return 0 <= r && r < H && 0 <= c && c < W;
    }
    static boolean isFree(int v) { return v == 0; }
    static boolean isUnknown(int v) { return v == -1; }
    static boolean isOccupied(int v) { return v >= 50; }

    static List<Coord> neighbors4(int H, int W, Coord u) {
        int[] dr = {-1, 1, 0, 0};
        int[] dc = {0, 0, -1, 1};
        List<Coord> out = new ArrayList<>();
        for (int k = 0; k < 4; k++) {
            int rr = u.r + dr[k], cc = u.c + dc[k];
            if (inBounds(H, W, rr, cc)) out.add(new Coord(rr, cc));
        }
        return out;
    }

    static List<Coord> neighbors8(int H, int W, Coord u) {
        List<Coord> out = new ArrayList<>();
        for (int dr = -1; dr <= 1; dr++) {
            for (int dc = -1; dc <= 1; dc++) {
                if (dr == 0 && dc == 0) continue;
                int rr = u.r + dr, cc = u.c + dc;
                if (inBounds(H, W, rr, cc)) out.add(new Coord(rr, cc));
            }
        }
        return out;
    }

    static List<Coord> detectFrontiers(int[][] grid) {
        int H = grid.length, W = grid[0].length;
        List<Coord> frontiers = new ArrayList<>();
        for (int r = 0; r < H; r++) {
            for (int c = 0; c < W; c++) {
                if (!isFree(grid[r][c])) continue;
                for (Coord v : neighbors4(H, W, new Coord(r, c))) {
                    if (isUnknown(grid[v.r][v.c])) { frontiers.add(new Coord(r, c)); break; }
                }
            }
        }
        return frontiers;
    }

    static List<List<Coord>> clusterFrontiers(int[][] grid, List<Coord> frontierCells) {
        int H = grid.length, W = grid[0].length;
        HashSet<Coord> frontierSet = new HashSet<>(frontierCells);
        HashSet<Coord> visited = new HashSet<>();
        List<List<Coord>> clusters = new ArrayList<>();

        for (Coord cell : frontierCells) {
            if (visited.contains(cell)) continue;
            if (!frontierSet.contains(cell)) continue;

            ArrayDeque<Coord> q = new ArrayDeque<>();
            q.add(cell);
            visited.add(cell);

            List<Coord> comp = new ArrayList<>();
            while (!q.isEmpty()) {
                Coord u = q.removeFirst();
                comp.add(u);
                for (Coord v : neighbors8(H, W, u)) {
                    if (frontierSet.contains(v) && !visited.contains(v)) {
                        visited.add(v);
                        q.addLast(v);
                    }
                }
            }
            clusters.add(comp);
        }

        clusters.sort((a, b) -> Integer.compare(b.size(), a.size()));
        return clusters;
    }

    static double[][] bfsDistFree(int[][] grid, Coord start) {
        int H = grid.length, W = grid[0].length;
        double INF = Double.POSITIVE_INFINITY;
        double[][] dist = new double[H][W];
        for (int r = 0; r < H; r++) Arrays.fill(dist[r], INF);
        if (!isFree(grid[start.r][start.c])) return dist;

        ArrayDeque<Coord> q = new ArrayDeque<>();
        q.add(start);
        dist[start.r][start.c] = 0.0;

        while (!q.isEmpty()) {
            Coord u = q.removeFirst();
            double du = dist[u.r][u.c];
            for (Coord v : neighbors4(H, W, u)) {
                if (!isFree(grid[v.r][v.c])) continue;
                if (Double.isInfinite(dist[v.r][v.c])) {
                    dist[v.r][v.c] = du + 1.0;
                    q.addLast(v);
                }
            }
        }
        return dist;
    }

    static Coord roundedCentroid(List<Coord> comp) {
        double sr = 0.0, sc = 0.0;
        for (Coord c : comp) { sr += c.r; sc += c.c; }
        double cr = sr / Math.max(1.0, (double)comp.size());
        double cc = sc / Math.max(1.0, (double)comp.size());
        return new Coord((int)Math.round(cr), (int)Math.round(cc));
    }

    static Coord chooseGoalNearCentroid(int[][] grid, double[][] dist, List<Coord> comp, int maxRadius) {
        int H = grid.length, W = grid[0].length;
        Coord c0 = roundedCentroid(comp);
        Coord best = null;
        double bestD = Double.POSITIVE_INFINITY;

        for (int rad = 0; rad <= maxRadius; rad++) {
            for (int r = c0.r - rad; r <= c0.r + rad; r++) {
                for (int c = c0.c - rad; c <= c0.c + rad; c++) {
                    if (!inBounds(H, W, r, c)) continue;
                    if (!isFree(grid[r][c])) continue;
                    double d = dist[r][c];
                    if (Double.isInfinite(d)) continue;
                    if (d < bestD) { bestD = d; best = new Coord(r, c); }
                }
            }
            if (best != null) return best;
        }
        return best;
    }

    static double scoreCluster(double distance, int size, double alpha, double beta) {
        return alpha * distance - beta * (double)size;
    }

    public static void main(String[] args) {
        int H = 30, W = 40;
        int[][] grid = new int[H][W];
        for (int r = 0; r < H; r++) Arrays.fill(grid[r], -1);

        for (int r = 5; r < 15; r++) for (int c = 5; c < 18; c++) grid[r][c] = 0;
        for (int r = 5; r < 15; r++) grid[r][12] = 100;
        for (int c = 12; c < 30; c++) grid[10][c] = 100;
        for (int r = 18; r < 25; r++) for (int c = 25; c < 35; c++) grid[r][c] = 0;

        Coord robot = new Coord(8, 8);

        List<Coord> frontiers = detectFrontiers(grid);
        List<List<Coord>> clusters = clusterFrontiers(grid, frontiers);
        double[][] dist = bfsDistFree(grid, robot);

        Coord bestGoal = null;
        double bestCost = Double.POSITIVE_INFINITY;

        for (List<Coord> comp : clusters) {
            if (comp.size() < 6) continue;
            Coord goal = chooseGoalNearCentroid(grid, dist, comp, 6);
            if (goal == null) continue;
            double d = dist[goal.r][goal.c];
            double J = scoreCluster(d, comp.size(), 1.0, 2.5);
            if (J < bestCost) { bestCost = J; bestGoal = goal; }
        }

        System.out.println("robot=(" + robot.r + "," + robot.c + ")");
        System.out.println("frontier_clusters=" + clusters.size());
        if (bestGoal != null) {
            System.out.println("best_goal=(" + bestGoal.r + "," + bestGoal.c + "), cost=" + bestCost);
        } else {
            System.out.println("No reachable frontier goal found.");
        }
    }
}
