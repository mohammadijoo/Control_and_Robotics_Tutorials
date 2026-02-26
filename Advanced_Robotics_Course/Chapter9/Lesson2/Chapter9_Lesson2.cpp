#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <tuple>

enum class Mode { Transit, Transfer };

struct HybridState {
    Mode mode;
    int rx, ry;  // robot position
    int ox, oy;  // object position

    bool operator==(const HybridState& other) const {
        return mode == other.mode &&
               rx == other.rx && ry == other.ry &&
               ox == other.ox && oy == other.oy;
    }
};

struct HybridStateHash {
    std::size_t operator()(HybridState const& s) const noexcept {
        std::size_t h1 = std::hash<int>{}(static_cast<int>(s.mode));
        std::size_t h2 = std::hash<int>{}(s.rx);
        std::size_t h3 = std::hash<int>{}(s.ry);
        std::size_t h4 = std::hash<int>{}(s.ox);
        std::size_t h5 = std::hash<int>{}(s.oy);
        return (((h1 ^ (h2 << 1)) ^ (h3 << 1)) ^ (h4 << 1)) ^ (h5 << 1);
    }
};

const int W = 7;
const int H = 7;
int grid[H][W];

bool inBounds(int x, int y) {
    return 0 <= x && x < W && 0 <= y && y < H;
}

bool isFree(int x, int y) {
    return inBounds(x, y) && grid[y][x] == 0;
}

std::vector<std::pair<std::string, HybridState>>
neighbors(const HybridState& s, std::pair<int,int> goalObj) {
    std::vector<std::pair<std::string, HybridState>> succ;

    // 4-connected motion
    const int dirs[4][2] = { {1,0},{-1,0},{0,1},{0,-1} };
    for (auto& d : dirs) {
        int nx = s.rx + d[0];
        int ny = s.ry + d[1];
        if (!isFree(nx, ny)) continue;

        HybridState ns;
        if (s.mode == Mode::Transit) {
            ns = {Mode::Transit, nx, ny, s.ox, s.oy};
        } else {
            ns = {Mode::Transfer, nx, ny, nx, ny};
        }
        succ.push_back({"move", ns});
    }

    // pick
    if (s.mode == Mode::Transit && s.rx == s.ox && s.ry == s.oy) {
        HybridState ns{Mode::Transfer, s.rx, s.ry, s.rx, s.ry};
        succ.push_back({"pick", ns});
    }

    // place
    if (s.mode == Mode::Transfer &&
        s.rx == goalObj.first && s.ry == goalObj.second) {
        HybridState ns{Mode::Transit, s.rx, s.ry, s.rx, s.ry};
        succ.push_back({"place", ns});
    }

    return succ;
}

int main() {
    // initialize grid (0 = free, 1 = obstacle)
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            grid[y][x] = 0;
    grid[3][3] = 1;

    std::pair<int,int> startRobot{0,0};
    std::pair<int,int> startObj{2,2};
    std::pair<int,int> goalObj{6,6};

    HybridState start{Mode::Transit, startRobot.first, startRobot.second,
                      startObj.first, startObj.second};

    std::queue<HybridState> frontier;
    frontier.push(start);

    std::unordered_map<HybridState,
        std::pair<HybridState, std::string>, HybridStateHash> parent;
    parent[start] = {start, "start"};

    auto isGoal = [&goalObj](const HybridState& s) {
        return s.mode == Mode::Transit &&
               s.ox == goalObj.first && s.oy == goalObj.second;
    };

    bool found = false;
    HybridState goal = start;

    while (!frontier.empty()) {
        HybridState s = frontier.front();
        frontier.pop();

        if (isGoal(s)) {
            found = true;
            goal = s;
            break;
        }

        for (auto& [act, ns] : neighbors(s, goalObj)) {
            if (parent.find(ns) == parent.end()) {
                parent[ns] = {s, act};
                frontier.push(ns);
            }
        }
    }

    if (!found) {
        std::cout << "No hybrid plan found.\n";
        return 0;
    }

    // Reconstruct and print plan
    std::vector<std::pair<std::string, HybridState>> plan;
    HybridState cur = goal;
    while (!(cur == start)) {
        auto it = parent.find(cur);
        const HybridState& par = it->second.first;
        const std::string& act = it->second.second;
        plan.push_back({act, cur});
        cur = par;
    }
    std::reverse(plan.begin(), plan.end());

    std::cout << "Plan length: " << plan.size() << "\n";
    for (auto& step : plan) {
        std::cout << step.first << " : "
                  << "mode=" << (step.second.mode == Mode::Transit ? "T" : "C")
                  << " r=(" << step.second.rx << "," << step.second.ry << ")"
                  << " o=(" << step.second.ox << "," << step.second.oy << ")"
                  << "\n";
    }

    return 0;
}
      
