#include <vector>
#include <cmath>
#include <limits>

static const int UNKNOWN = -1;
static const int FREE = 0;
static const int OBSTACLE = 1;

bool is_frontier(const std::vector<std::vector<int>>& grid,
                 int i, int j) {
    if (grid[i][j] != FREE) return false;
    const int di[4] = {-1, 1, 0, 0};
    const int dj[4] = {0, 0, -1, 1};
    int rows = static_cast<int>(grid.size());
    int cols = static_cast<int>(grid[0].size());
    for (int k = 0; k < 4; ++k) {
        int ni = i + di[k];
        int nj = j + dj[k];
        if (0 <= ni && ni < rows && 0 <= nj && nj < cols) {
            if (grid[ni][nj] == UNKNOWN) return true;
        }
    }
    return false;
}

std::pair<int,int> select_closest_frontier(
        const std::vector<std::vector<int>>& grid,
        int ri, int rj,
        bool& found) {
    found = false;
    double bestDist = std::numeric_limits<double>::infinity();
    std::pair<int,int> best(-1, -1);
    int rows = static_cast<int>(grid.size());
    int cols = static_cast<int>(grid[0].size());
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (is_frontier(grid, i, j)) {
                double dx = static_cast<double>(i - ri);
                double dy = static_cast<double>(j - rj);
                double d = std::sqrt(dx*dx + dy*dy);
                if (d < bestDist) {
                    bestDist = d;
                    best = std::make_pair(i, j);
                    found = true;
                }
            }
        }
    }
    return best;
}
      
