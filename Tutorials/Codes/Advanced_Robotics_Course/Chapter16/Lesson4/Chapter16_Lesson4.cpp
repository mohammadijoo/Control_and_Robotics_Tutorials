#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>

using AdjList = std::vector<std::vector<int>>;

std::unordered_set<int> forwardReach(const AdjList& adj,
                                      const std::unordered_set<int>& S0) {
    std::unordered_set<int> reachable = S0;
    std::queue<int> q;
    for (int s : S0) q.push(s);

    while (!q.empty()) {
        int s = q.front();
        q.pop();
        for (int s_next : adj[s]) {
            if (reachable.find(s_next) == reachable.end()) {
                reachable.insert(s_next);
                q.push(s_next);
            }
        }
    }
    return reachable;
}

std::unordered_set<int> backwardReach(const AdjList& adj,
                                       const std::unordered_set<int>& Bad) {
    int N = static_cast<int>(adj.size());
    AdjList radj(N);
    for (int s = 0; s < N; ++s) {
        for (int s_next : adj[s]) {
            radj[s_next].push_back(s);
        }
    }

    std::unordered_set<int> backward = Bad;
    std::queue<int> q;
    for (int s : Bad) q.push(s);

    while (!q.empty()) {
        int s = q.front();
        q.pop();
        for (int s_prev : radj[s]) {
            if (backward.find(s_prev) == backward.end()) {
                backward.insert(s_prev);
                q.push(s_prev);
            }
        }
    }
    return backward;
}

int main() {
    AdjList adj = {
        {1, 2},    // 0
        {3},       // 1
        {3, 4},    // 2
        {5},       // 3
        {5, 6},    // 4
        {7},       // 5
        {7},       // 6
        {}         // 7
    };

    std::unordered_set<int> S0 = {0};
    std::unordered_set<int> Bad = {6};

    auto R_star = forwardReach(adj, S0);
    auto B_star = backwardReach(adj, Bad);

    std::cout << "Reachable from S0: ";
    for (int s : R_star) std::cout << s << " ";
    std::cout << std::endl;

    std::cout << "Backward reachable to Bad: ";
    for (int s : B_star) std::cout << s << " ";
    std::cout << std::endl;

    bool safe = (S0.size() == 0);
    for (int s0 : S0) {
        if (B_star.find(s0) != B_star.end()) {
            safe = false;
        }
    }
    std::cout << "Safety holds? " << (safe ? "true" : "false") << std::endl;
    return 0;
}
      
