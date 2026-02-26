#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
using namespace std;

int main() {
    const int S = 21;                 // positions -10,...,10
    const int A = 3;                  // actions: 0 left, 1 stay, 2 right
    vector<vector<double>> Q(S, vector<double>(A, 0.0));

    double alpha = 0.1, gamma = 0.95, eps = 0.2;
    mt19937 gen(0);
    uniform_real_distribution<double> uni(0.0,1.0);
    uniform_int_distribution<int> a_rand(0,A-1);

    auto step = [&](int s, int a){
        int pos = s - 10;
        int next_pos = pos + (a-1); // left=-1, stay=0, right=+1
        next_pos = max(-10, min(10, next_pos));
        int s2 = next_pos + 10;
        double r = (next_pos == 0) ? 0.0 : -1.0;
        return pair<int,double>(s2,r);
    };

    for(int ep=0; ep<5000; ++ep){
        int s = 10+5; // start at pos=5
        for(int t=0; t<50; ++t){
            int a;
            if(uni(gen) < eps) a = a_rand(gen);
            else a = int(max_element(Q[s].begin(), Q[s].end()) - Q[s].begin());

            auto sr = step(s,a);
            int s2 = sr.first;
            double r = sr.second;

            double qmax = *max_element(Q[s2].begin(), Q[s2].end());
            Q[s][a] += alpha * (r + gamma*qmax - Q[s][a]);
            s = s2;
            if(s==10) break; // reached origin
        }
    }

    cout << "Greedy policy (pos -> action):\n";
    for(int s=0; s<S; ++s){
        int pos = s-10;
        int a = int(max_element(Q[s].begin(), Q[s].end()) - Q[s].begin());
        cout << pos << " -> " << (a-1) << "\n";
    }
}
      