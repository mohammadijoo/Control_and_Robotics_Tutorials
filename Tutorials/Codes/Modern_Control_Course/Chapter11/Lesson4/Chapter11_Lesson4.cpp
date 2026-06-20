// Chapter11_Lesson4.cpp
// Structural controllability graph test for xdot = A x + B u.
// Pattern entries: 1 = free structural nonzero, 0 = fixed zero.
// Compile:
//   g++ -std=c++17 -O2 Chapter11_Lesson4.cpp -o Chapter11_Lesson4

#include <iostream>
#include <vector>
#include <queue>
#include <stdexcept>
#include <string>

using Matrix = std::vector<std::vector<int>>;

struct Result {
    bool reachable;
    bool fullMatching;
    bool structurallyControllable;
    int matchingSize;
    std::vector<bool> reachableStates;
};

void validatePattern(const Matrix& Abar, const Matrix& Bbar, int& n, int& m) {
    n = static_cast<int>(Abar.size());
    if (n == 0) throw std::runtime_error("Abar must be nonempty.");
    for (const auto& row : Abar) {
        if (static_cast<int>(row.size()) != n) throw std::runtime_error("Abar must be square.");
    }
    if (static_cast<int>(Bbar.size()) != n) throw std::runtime_error("Bbar row count must match Abar.");
    m = static_cast<int>(Bbar[0].size());
    if (m == 0) throw std::runtime_error("Bbar must have at least one input column.");
    for (const auto& row : Bbar) {
        if (static_cast<int>(row.size()) != m) throw std::runtime_error("Bbar rows must have equal length.");
    }
}

std::vector<bool> inputReachability(const Matrix& Abar, const Matrix& Bbar) {
    int n, m;
    validatePattern(Abar, Bbar, n, m);

    std::vector<std::vector<int>> adj(n + m);

    // Abar[i][j] means x_j influences xdot_i, so x_j -> x_i.
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            if (Abar[i][j] != 0) adj[j].push_back(i);

    // Bbar[i][k] means u_k influences xdot_i, so u_k -> x_i.
    for (int i = 0; i < n; ++i)
        for (int k = 0; k < m; ++k)
            if (Bbar[i][k] != 0) adj[n + k].push_back(i);

    std::vector<bool> seen(n + m, false);
    std::queue<int> q;
    for (int k = 0; k < m; ++k) {
        seen[n + k] = true;
        q.push(n + k);
    }

    while (!q.empty()) {
        int v = q.front();
        q.pop();
        for (int w : adj[v]) {
            if (!seen[w]) {
                seen[w] = true;
                q.push(w);
            }
        }
    }

    return std::vector<bool>(seen.begin(), seen.begin() + n);
}

class HopcroftKarp {
public:
    HopcroftKarp(int leftCount, int rightCount, std::vector<std::vector<int>> adj)
        : L(leftCount), R(rightCount), adjLeft(std::move(adj)),
          pairU(L, -1), pairV(R, -1), dist(L, -1) {}

    int maximumMatching() {
        int matching = 0;
        while (bfs()) {
            for (int u = 0; u < L; ++u) {
                if (pairU[u] == -1 && dfs(u)) ++matching;
            }
        }
        return matching;
    }

private:
    int L, R;
    std::vector<std::vector<int>> adjLeft;
    std::vector<int> pairU, pairV, dist;

    bool bfs() {
        std::queue<int> q;
        bool foundFreeRight = false;
        for (int u = 0; u < L; ++u) {
            if (pairU[u] == -1) {
                dist[u] = 0;
                q.push(u);
            } else {
                dist[u] = -1;
            }
        }

        while (!q.empty()) {
            int u = q.front();
            q.pop();
            for (int v : adjLeft[u]) {
                int mate = pairV[v];
                if (mate == -1) {
                    foundFreeRight = true;
                } else if (dist[mate] == -1) {
                    dist[mate] = dist[u] + 1;
                    q.push(mate);
                }
            }
        }
        return foundFreeRight;
    }

    bool dfs(int u) {
        for (int v : adjLeft[u]) {
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
};

int maximumPatternMatchingSize(const Matrix& Abar, const Matrix& Bbar) {
    int n, m;
    validatePattern(Abar, Bbar, n, m);
    int leftCount = n + m;
    int rightCount = n;
    std::vector<std::vector<int>> adjLeft(leftCount);

    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            if (Abar[i][j] != 0) adjLeft[j].push_back(i);

    for (int i = 0; i < n; ++i)
        for (int k = 0; k < m; ++k)
            if (Bbar[i][k] != 0) adjLeft[n + k].push_back(i);

    HopcroftKarp hk(leftCount, rightCount, adjLeft);
    return hk.maximumMatching();
}

Result structuralControllability(const Matrix& Abar, const Matrix& Bbar) {
    int n, m;
    validatePattern(Abar, Bbar, n, m);
    auto reachableStates = inputReachability(Abar, Bbar);
    bool allReachable = true;
    for (bool flag : reachableStates) allReachable = allReachable && flag;

    int matchingSize = maximumPatternMatchingSize(Abar, Bbar);
    bool fullMatching = (matchingSize == n);

    return {allReachable, fullMatching, allReachable && fullMatching, matchingSize, reachableStates};
}

void printResult(const std::string& title, const Result& r) {
    std::cout << title << "\n";
    std::cout << "  all states input reachable: " << (r.reachable ? "true" : "false") << "\n";
    std::cout << "  matching size: " << r.matchingSize << "\n";
    std::cout << "  no dilation via full row matching: " << (r.fullMatching ? "true" : "false") << "\n";
    std::cout << "  structurally controllable: " << (r.structurallyControllable ? "true" : "false") << "\n";
}

int main() {
    Matrix Achain = {
        {0, 0, 0},
        {1, 0, 0},
        {0, 1, 0}
    };
    Matrix Bchain = {
        {1},
        {0},
        {0}
    };

    Matrix Adilation = {
        {0, 0, 0},
        {1, 0, 0},
        {1, 0, 0}
    };
    Matrix Bdilation = {
        {1},
        {0},
        {0}
    };

    printResult("Chain example", structuralControllability(Achain, Bchain));
    printResult("Dilation example", structuralControllability(Adilation, Bdilation));
    return 0;
}
