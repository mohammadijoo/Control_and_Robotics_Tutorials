/*
Chapter 1 — Lesson 3: Environment Representations for AMR
File: Chapter1_Lesson3.java

Java implementation of:
  1) Occupancy grid (rectangles)
  2) Brushfire distance transform (Manhattan, multi-source BFS)
  3) Signed distance field (SDF) and disc-robot inflation

Compile:
  javac Chapter1_Lesson3.java
Run:
  java Chapter1_Lesson3
*/

import java.io.FileWriter;
import java.io.IOException;

public class Chapter1_Lesson3 {

    static int idx(int r, int c, int W) { return r * W + c; }

    static void fillRect(byte[] occ, int H, int W, int r0, int c0, int r1, int c1) {
        if (r0 < 0) r0 = 0;
        if (c0 < 0) c0 = 0;
        if (r1 >= H) r1 = H - 1;
        if (c1 >= W) c1 = W - 1;
        for (int r = r0; r <= r1; r++) {
            for (int c = c0; c <= c1; c++) {
                occ[idx(r, c, W)] = 1;
            }
        }
    }

    static void brushfireManhattan(byte[] occ, int H, int W, int[] dist) {
        final int INF = 1000000000;
        final int N = H * W;
        for (int i = 0; i < N; i++) dist[i] = INF;

        int[] q = new int[N];
        int head = 0, tail = 0;

        for (int r = 0; r < H; r++) {
            for (int c = 0; c < W; c++) {
                int k = idx(r, c, W);
                if (occ[k] == 1) {
                    dist[k] = 0;
                    q[tail++] = k;
                }
            }
        }

        int[] dr = new int[]{-1, 1, 0, 0};
        int[] dc = new int[]{0, 0, -1, 1};

        while (head < tail) {
            int k = q[head++];
            int r = k / W;
            int c = k - r * W;
            int d = dist[k];

            for (int t = 0; t < 4; t++) {
                int rr = r + dr[t];
                int cc = c + dc[t];
                if (rr >= 0 && rr < H && cc >= 0 && cc < W) {
                    int kk = idx(rr, cc, W);
                    if (dist[kk] > d + 1) {
                        dist[kk] = d + 1;
                        q[tail++] = kk;
                    }
                }
            }
        }
    }

    static void writeCsv(String filename, byte[] grid, int H, int W) throws IOException {
        FileWriter fw = new FileWriter(filename);
        for (int r = 0; r < H; r++) {
            for (int c = 0; c < W; c++) {
                fw.write(Integer.toString(grid[idx(r, c, W)]));
                if (c + 1 < W) fw.write(",");
            }
            fw.write("\n");
        }
        fw.close();
    }

    public static void main(String[] args) throws Exception {
        final int H = 160;
        final int W = 200;
        final double res = 0.05;

        byte[] occ = new byte[H * W];
        byte[] occInv = new byte[H * W];
        int[] dToObs = new int[H * W];
        int[] dToFree = new int[H * W];
        double[] sdf = new double[H * W];
        byte[] occInfl = new byte[H * W];

        // Obstacles
        fillRect(occ, H, W, 30, 40, 55, 100);
        fillRect(occ, H, W, 90, 120, 130, 180);
        fillRect(occ, H, W, 110, 20, 140, 45);

        // Distances
        brushfireManhattan(occ, H, W, dToObs);
        for (int i = 0; i < H * W; i++) occInv[i] = (byte)(1 - occ[i]);
        brushfireManhattan(occInv, H, W, dToFree);

        // SDF
        for (int i = 0; i < H * W; i++) {
            if (occ[i] == 1) sdf[i] = -((double)dToFree[i]) * res;
            else sdf[i] = ((double)dToObs[i]) * res;
        }

        // Inflation
        final double robotRadius = 0.30;
        for (int i = 0; i < H * W; i++) {
            occInfl[i] = (byte)(sdf[i] < robotRadius ? 1 : 0);
        }

        // CSV export
        writeCsv("occupancy_grid_java.csv", occ, H, W);
        writeCsv("inflated_occupancy_grid_java.csv", occInfl, H, W);

        // Summary
        int free0 = 0, free1 = 0;
        for (int i = 0; i < H * W; i++) {
            if (occ[i] == 0) free0++;
            if (occInfl[i] == 0) free1++;
        }

        System.out.println("Grid: H=" + H + ", W=" + W + ", res=" + res + " m");
        System.out.println("Free ratio (original): " + ((double)free0 / (double)(H * W)));
        System.out.println("Free ratio (inflated): " + ((double)free1 / (double)(H * W)));
        System.out.println("Wrote: occupancy_grid_java.csv, inflated_occupancy_grid_java.csv");
    }
}
