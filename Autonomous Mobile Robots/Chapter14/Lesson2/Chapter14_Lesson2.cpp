// Chapter14_Lesson2.cpp
// Costmaps and Inflation Concepts — simple grid + brushfire inflation (no ROS deps).
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <limits>
#include <vector>

struct Cell { int x, y; };

static std::vector<std::vector<int>> makeToyMap(int w=60, int h=40) {
    std::vector<std::vector<int>> g(h, std::vector<int>(w, 0));
    for (int x=0; x<w; ++x) { g[0][x]=1; g[h-1][x]=1; }
    for (int y=0; y<h; ++y) { g[y][0]=1; g[y][w-1]=1; }
    for (int x=10; x<50; ++x) g[12][x]=1;
    for (int y=18; y<33; ++y) g[y][28]=1;
    for (int x=35; x<55; ++x) g[28][x]=1;
    return g;
}

static std::vector<std::vector<double>> brushfireDistance(
    const std::vector<std::vector<int>>& obs, double resolution)
{
    const int h = (int)obs.size();
    const int w = (int)obs[0].size();
    const double INF = std::numeric_limits<double>::infinity();

    std::vector<std::vector<double>> dist(h, std::vector<double>(w, INF));
    std::deque<Cell> q;

    for (int y=0; y<h; ++y) for (int x=0; x<w; ++x) {
        if (obs[y][x] == 1) { dist[y][x] = 0.0; q.push_back({x,y}); }
    }

    struct Nbr { int dx, dy; double step; };
    const double s2 = std::sqrt(2.0);
    const Nbr nbrs[8] = {
        {-1,0,1.0},{1,0,1.0},{0,-1,1.0},{0,1,1.0},
        {-1,-1,s2},{1,-1,s2},{-1,1,s2},{1,1,s2}
    };

    while (!q.empty()) {
        Cell c = q.front(); q.pop_front();
        const double dxy = dist[c.y][c.x];
        for (const auto& nb : nbrs) {
            const int nx = c.x + nb.dx;
            const int ny = c.y + nb.dy;
            if (0 <= nx && nx < w && 0 <= ny && ny < h) {
                const double nd = dxy + nb.step * resolution;
                if (nd < dist[ny][nx]) { dist[ny][nx] = nd; q.push_back({nx,ny}); }
            }
        }
    }
    return dist;
}

static int inflationCost(double d, double r_inscribed, double r_inflation, double cost_scaling,
                         int lethal_cost=254, int inscribed_cost=253)
{
    if (d <= 0.0) return lethal_cost;
    if (d <= r_inscribed) return inscribed_cost;
    if (d > r_inflation) return 0;

    const double c = (inscribed_cost - 1) * std::exp(-cost_scaling * (d - r_inscribed)) + 1.0;
    int ci = (int)std::llround(c);
    if (ci < 1) ci = 1;
    if (ci > inscribed_cost - 1) ci = inscribed_cost - 1;
    return ci;
}

static std::vector<std::vector<int>> buildInflatedCostmap(
    const std::vector<std::vector<int>>& obs,
    double resolution, double r_inscribed, double r_inflation, double cost_scaling)
{
    auto dist = brushfireDistance(obs, resolution);
    const int h = (int)obs.size();
    const int w = (int)obs[0].size();
    std::vector<std::vector<int>> cm(h, std::vector<int>(w, 0));
    for (int y=0; y<h; ++y) for (int x=0; x<w; ++x)
        cm[y][x] = inflationCost(dist[y][x], r_inscribed, r_inflation, cost_scaling);
    return cm;
}

static void asciiRender(const std::vector<std::vector<int>>& cm) {
    for (const auto& row : cm) {
        for (int c : row) {
            char ch = ' ';
            if (c >= 254) ch = '#';
            else if (c >= 200) ch = 'X';
            else if (c >= 120) ch = '+';
            else if (c >= 40)  ch = '.';
            std::cout << ch;
        }
        std::cout << "\n";
    }
}

int main() {
    const double resolution = 0.05;
    const double r_inscribed = 0.25;
    const double r_inflation = 0.60;
    const double cost_scaling = 6.0;

    auto obs = makeToyMap();
    auto cm = buildInflatedCostmap(obs, resolution, r_inscribed, r_inflation, cost_scaling);
    std::cout << "Inflated costmap (ASCII preview):\n";
    asciiRender(cm);
    return 0;
}
