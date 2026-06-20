/*
Chapter 1 — Lesson 3: Environment Representations for AMR
File: Chapter1_Lesson3.cpp

A minimal C++ (mostly C-style) implementation of:
  1) Occupancy grid from axis-aligned rectangular obstacles
  2) Multi-source BFS (Brushfire) distance transform (Manhattan distance)
  3) Signed distance field (SDF) and disc-robot inflation

No template types are used (to avoid angle brackets in HTML embedding).
Compile (example):
  g++ -O2 -std=c++17 Chapter1_Lesson3.cpp -o Chapter1_Lesson3
*/

#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "string.h"

static int idx(int r, int c, int W) { return r * W + c; }

static void brushfire_manhattan(const unsigned char* occ, int H, int W, int* dist) {
    const int INF = 1000000000;
    int N = H * W;

    for (int i = 0; i < N; ++i) dist[i] = INF;

    int* q = (int*)malloc((size_t)N * sizeof(int));
    int head = 0, tail = 0;

    for (int r = 0; r < H; ++r) {
        for (int c = 0; c < W; ++c) {
            int k = idx(r, c, W);
            if (occ[k] == 1) {
                dist[k] = 0;
                q[tail++] = k;
            }
        }
    }

    while (head < tail) {
        int k = q[head++];
        int r = k / W;
        int c = k - r * W;
        int d = dist[k];

        // 4-neighborhood
        const int dr[4] = {-1, 1, 0, 0};
        const int dc[4] = {0, 0, -1, 1};

        for (int t = 0; t < 4; ++t) {
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

    free(q);
}

static void fill_rect(unsigned char* occ, int H, int W,
                      int r0, int c0, int r1, int c1) {
    // Fill rectangle [r0,r1] x [c0,c1] inclusive, clamped
    if (r0 < 0) r0 = 0;
    if (c0 < 0) c0 = 0;
    if (r1 >= H) r1 = H - 1;
    if (c1 >= W) c1 = W - 1;
    for (int r = r0; r <= r1; ++r) {
        for (int c = c0; c <= c1; ++c) {
            occ[idx(r, c, W)] = 1;
        }
    }
}

int main() {
    // Grid parameters
    const int H = 160;        // rows
    const int W = 200;        // cols
    const double res = 0.05;  // meters per cell (5 cm)

    unsigned char* occ = (unsigned char*)malloc((size_t)H * (size_t)W);
    unsigned char* occ_inv = (unsigned char*)malloc((size_t)H * (size_t)W);
    int* d_to_obs = (int*)malloc((size_t)H * (size_t)W * sizeof(int));
    int* d_to_free = (int*)malloc((size_t)H * (size_t)W * sizeof(int));
    double* sdf = (double*)malloc((size_t)H * (size_t)W * sizeof(double));

    if (!occ || !occ_inv || !d_to_obs || !d_to_free || !sdf) {
        printf("Memory allocation failed.\n");
        return 1;
    }

    memset(occ, 0, (size_t)H * (size_t)W);

    // Rectangular obstacles in grid indices (row,col)
    fill_rect(occ, H, W, 30, 40, 55, 100);
    fill_rect(occ, H, W, 90, 120, 130, 180);
    fill_rect(occ, H, W, 110, 20, 140, 45);

    // Distances to obstacle for free cells
    brushfire_manhattan(occ, H, W, d_to_obs);

    // Distances to free space for obstacle cells (run on inverted grid)
    for (int i = 0; i < H * W; ++i) occ_inv[i] = (unsigned char)(1 - occ[i]);
    brushfire_manhattan(occ_inv, H, W, d_to_free);

    // Signed distance field in meters
    for (int i = 0; i < H * W; ++i) {
        if (occ[i] == 1) sdf[i] = -((double)d_to_free[i]) * res;
        else sdf[i] = ((double)d_to_obs[i]) * res;
    }

    // Inflate obstacles for disc robot radius
    const double robot_radius = 0.30;
    unsigned char* occ_infl = (unsigned char*)malloc((size_t)H * (size_t)W);
    if (!occ_infl) return 1;

    for (int i = 0; i < H * W; ++i) {
        occ_infl[i] = (unsigned char)(sdf[i] < robot_radius ? 1 : 0);
    }

    // Export CSV for inspection (optional)
    FILE* f = fopen("occupancy_grid_cpp.csv", "w");
    if (f) {
        for (int r = 0; r < H; ++r) {
            for (int c = 0; c < W; ++c) {
                fprintf(f, "%d%s", (int)occ[idx(r, c, W)], (c + 1 < W) ? "," : "");
            }
            fprintf(f, "\n");
        }
        fclose(f);
    }

    FILE* g = fopen("inflated_occupancy_grid_cpp.csv", "w");
    if (g) {
        for (int r = 0; r < H; ++r) {
            for (int c = 0; c < W; ++c) {
                fprintf(g, "%d%s", (int)occ_infl[idx(r, c, W)], (c + 1 < W) ? "," : "");
            }
            fprintf(g, "\n");
        }
        fclose(g);
    }

    // Small console summary
    int free0 = 0, free1 = 0;
    for (int i = 0; i < H * W; ++i) {
        if (occ[i] == 0) free0++;
        if (occ_infl[i] == 0) free1++;
    }
    printf("Grid: H=%d, W=%d, res=%.3f m\n", H, W, res);
    printf("Free ratio (original): %.4f\n", (double)free0 / (double)(H * W));
    printf("Free ratio (inflated): %.4f\n", (double)free1 / (double)(H * W));
    printf("Wrote: occupancy_grid_cpp.csv, inflated_occupancy_grid_cpp.csv\n");

    free(occ);
    free(occ_inv);
    free(d_to_obs);
    free(d_to_free);
    free(sdf);
    free(occ_infl);
    return 0;
}
