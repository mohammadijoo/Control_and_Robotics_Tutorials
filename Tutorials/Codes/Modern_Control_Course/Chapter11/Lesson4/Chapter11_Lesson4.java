// Chapter11_Lesson4.java
// Structural controllability graph test for xdot = A x + B u.
// Pattern entries: 1 = free structural nonzero, 0 = fixed zero.
// Compile and run:
//   javac Chapter11_Lesson4.java
//   java Chapter11_Lesson4

import java.util.*;

public class Chapter11_Lesson4 {
    static class Result {
        boolean reachable;
        boolean fullMatching;
        boolean structurallyControllable;
        int matchingSize;
        boolean[] reachableStates;

        Result(boolean reachable, boolean fullMatching, boolean structurallyControllable,
               int matchingSize, boolean[] reachableStates) {
            this.reachable = reachable;
            this.fullMatching = fullMatching;
            this.structurallyControllable = structurallyControllable;
            this.matchingSize = matchingSize;
            this.reachableStates = reachableStates;
        }
    }

    static int[] validatePattern(int[][] Abar, int[][] Bbar) {
        int n = Abar.length;
        if (n == 0) throw new IllegalArgumentException("Abar must be nonempty.");
        for (int[] row : Abar) {
            if (row.length != n) throw new IllegalArgumentException("Abar must be square.");
        }
        if (Bbar.length != n) throw new IllegalArgumentException("Bbar row count must match Abar.");
        int m = Bbar[0].length;
        if (m == 0) throw new IllegalArgumentException("Bbar must have at least one input column.");
        for (int[] row : Bbar) {
            if (row.length != m) throw new IllegalArgumentException("Bbar rows must have equal length.");
        }
        return new int[] {n, m};
    }

    static boolean[] inputReachability(int[][] Abar, int[][] Bbar) {
        int[] dim = validatePattern(Abar, Bbar);
        int n = dim[0];
        int m = dim[1];

        List<List<Integer>> adj = new ArrayList<>();
        for (int v = 0; v < n + m; v++) adj.add(new ArrayList<>());

        // Abar[i][j] means x_j influences xdot_i, so x_j -> x_i.
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                if (Abar[i][j] != 0) adj.get(j).add(i);

        // Bbar[i][k] means u_k influences xdot_i, so u_k -> x_i.
        for (int i = 0; i < n; i++)
            for (int k = 0; k < m; k++)
                if (Bbar[i][k] != 0) adj.get(n + k).add(i);

        boolean[] seen = new boolean[n + m];
        ArrayDeque<Integer> q = new ArrayDeque<>();
        for (int k = 0; k < m; k++) {
            seen[n + k] = true;
            q.add(n + k);
        }

        while (!q.isEmpty()) {
            int v = q.remove();
            for (int w : adj.get(v)) {
                if (!seen[w]) {
                    seen[w] = true;
                    q.add(w);
                }
            }
        }

        return Arrays.copyOf(seen, n);
    }

    static class HopcroftKarp {
        int L, R;
        List<List<Integer>> adjLeft;
        int[] pairU, pairV, dist;

        HopcroftKarp(int leftCount, int rightCount, List<List<Integer>> adjLeft) {
            this.L = leftCount;
            this.R = rightCount;
            this.adjLeft = adjLeft;
            this.pairU = new int[L];
            this.pairV = new int[R];
            this.dist = new int[L];
            Arrays.fill(pairU, -1);
            Arrays.fill(pairV, -1);
        }

        boolean bfs() {
            ArrayDeque<Integer> q = new ArrayDeque<>();
            boolean foundFreeRight = false;

            for (int u = 0; u < L; u++) {
                if (pairU[u] == -1) {
                    dist[u] = 0;
                    q.add(u);
                } else {
                    dist[u] = -1;
                }
            }

            while (!q.isEmpty()) {
                int u = q.remove();
                for (int v : adjLeft.get(u)) {
                    int mate = pairV[v];
                    if (mate == -1) {
                        foundFreeRight = true;
                    } else if (dist[mate] == -1) {
                        dist[mate] = dist[u] + 1;
                        q.add(mate);
                    }
                }
            }
            return foundFreeRight;
        }

        boolean dfs(int u) {
            for (int v : adjLeft.get(u)) {
                int mate = pairV[v];
                if (mate == -1 || (dist[mate] == dist[u] + 1 && dfs(mate))) {
                    pairU[u] = v;
                    pairV[v] = u;
                    return true;
                }
            }
            dist[u] = -1;
            return false;
        }

        int maximumMatching() {
            int matching = 0;
            while (bfs()) {
                for (int u = 0; u < L; u++) {
                    if (pairU[u] == -1 && dfs(u)) matching++;
                }
            }
            return matching;
        }
    }

    static int maximumPatternMatchingSize(int[][] Abar, int[][] Bbar) {
        int[] dim = validatePattern(Abar, Bbar);
        int n = dim[0];
        int m = dim[1];

        int leftCount = n + m;
        int rightCount = n;
        List<List<Integer>> adjLeft = new ArrayList<>();
        for (int u = 0; u < leftCount; u++) adjLeft.add(new ArrayList<>());

        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                if (Abar[i][j] != 0) adjLeft.get(j).add(i);

        for (int i = 0; i < n; i++)
            for (int k = 0; k < m; k++)
                if (Bbar[i][k] != 0) adjLeft.get(n + k).add(i);

        return new HopcroftKarp(leftCount, rightCount, adjLeft).maximumMatching();
    }

    static Result structuralControllability(int[][] Abar, int[][] Bbar) {
        int[] dim = validatePattern(Abar, Bbar);
        int n = dim[0];

        boolean[] reach = inputReachability(Abar, Bbar);
        boolean allReachable = true;
        for (boolean flag : reach) allReachable = allReachable && flag;

        int matchingSize = maximumPatternMatchingSize(Abar, Bbar);
        boolean fullMatching = matchingSize == n;

        return new Result(allReachable, fullMatching, allReachable && fullMatching, matchingSize, reach);
    }

    static void printResult(String title, Result r) {
        System.out.println(title);
        System.out.println("  all states input reachable: " + r.reachable);
        System.out.println("  matching size: " + r.matchingSize);
        System.out.println("  no dilation via full row matching: " + r.fullMatching);
        System.out.println("  structurally controllable: " + r.structurallyControllable);
    }

    public static void main(String[] args) {
        int[][] Achain = {
            {0, 0, 0},
            {1, 0, 0},
            {0, 1, 0}
        };
        int[][] Bchain = {
            {1},
            {0},
            {0}
        };

        int[][] Adilation = {
            {0, 0, 0},
            {1, 0, 0},
            {1, 0, 0}
        };
        int[][] Bdilation = {
            {1},
            {0},
            {0}
        };

        printResult("Chain example", structuralControllability(Achain, Bchain));
        printResult("Dilation example", structuralControllability(Adilation, Bdilation));
    }
}
