// Chapter17_Lesson4.cpp
// Exploration Under Limited Battery/Time (Budget-Aware Frontier Exploration)
// Self-contained demo without ROS.
// Build: g++ -O2 -std=c++17 Chapter17_Lesson4.cpp -o demo
#include <bits/stdc++.h>
using namespace std;

struct Budget { double E, T; };
struct RobotParams { double E_step=1.2, T_step=1.0, E_base=0.10, E_reserve=15.0; };
struct SensorParams { int r=3; };

static inline double clampd(double x,double lo,double hi){ return max(lo,min(hi,x)); }

static double entropyBern(double p){
    p = clampd(p, 1e-9, 1.0-1e-9);
    return -(p*log(p) + (1.0-p)*log(1.0-p));
}

struct Grid {
    int W,H;
    vector<double> p; // row-major p_occ
    Grid(int w,int h):W(w),H(h),p(w*h,0.5){}
    bool inb(int x,int y) const { return 0<=x && x<W && 0<=y && y<H; }
    double get(int x,int y) const { return p[y*W+x]; }
    void setv(int x,int y,double v){ p[y*W+x]=clampd(v,0.0,1.0); }
    bool isFree(int x,int y,double thr=0.65) const { return get(x,y) < thr; }
    bool isOcc(int x,int y,double thr=0.65) const { return get(x,y) > thr; }
};

using Cell = pair<int,int>;

static vector<Cell> neigh4(const Grid& g, Cell c){
    int x=c.first, y=c.second;
    vector<Cell> cand={{x+1,y},{x-1,y},{x,y+1},{x,y-1}};
    vector<Cell> out;
    for(auto &q:cand) if(g.inb(q.first,q.second)) out.push_back(q);
    return out;
}

static vector<Cell> astar(const Grid& g, Cell s, Cell goal, double occ_thr=0.65){
    if(!g.inb(s.first,s.second) || !g.inb(goal.first,goal.second)) return {};
    if(!g.isFree(s.first,s.second,occ_thr) || !g.isFree(goal.first,goal.second,occ_thr)) return {};
    auto h = [&](Cell c){ return abs(c.first-goal.first)+abs(c.second-goal.second); };

    unordered_map<long long,double> gscore;
    unordered_map<long long, long long> parent;
    auto key=[&](Cell c)->long long{ return (long long)c.second*g.W + c.first; };

    struct Node{ double f; Cell c; };
    struct Cmp{ bool operator()(const Node& a,const Node& b) const { return a.f>b.f; } };
    priority_queue<Node, vector<Node>, Cmp> pq;
    pq.push({(double)h(s), s});
    gscore[key(s)] = 0.0;

    while(!pq.empty()){
        auto cur = pq.top().c; pq.pop();
        if(cur==goal){
            vector<Cell> path; path.push_back(cur);
            long long k=key(cur);
            while(parent.count(k)){
                k = parent[k];
                path.push_back({(int)(k%g.W), (int)(k/g.W)});
            }
            reverse(path.begin(), path.end());
            return path;
        }
        long long kc=key(cur);
        double gc=gscore[kc];
        for(auto nb: neigh4(g,cur)){
            if(!g.isFree(nb.first, nb.second, occ_thr)) continue;
            long long kn=key(nb);
            double tent = gc + 1.0;
            if(!gscore.count(kn) || tent < gscore[kn]){
                gscore[kn]=tent;
                parent[kn]=kc;
                pq.push({tent + (double)h(nb), nb});
            }
        }
    }
    return {};
}

static vector<Cell> detectFrontiers(const Grid& g, double free_thr=0.35){
    vector<Cell> fr;
    for(int y=0;y<g.H;y++){
        for(int x=0;x<g.W;x++){
            double p=g.get(x,y);
            if(p >= free_thr) continue;
            for(auto nb: neigh4(g,{x,y})){
                double pn=g.get(nb.first, nb.second);
                if(fabs(pn-0.5) < 0.15){ fr.push_back({x,y}); break; }
            }
        }
    }
    return fr;
}

static double expectedIG(const Grid& belief, Cell pose, const SensorParams& sp){
    double ig=0.0;
    for(int dy=-sp.r; dy<=sp.r; dy++){
        for(int dx=-sp.r; dx<=sp.r; dx++){
            int x=pose.first+dx, y=pose.second+dy;
            if(!belief.inb(x,y)) continue;
            double p0 = belief.get(x,y);
            double H0 = entropyBern(p0);
            // simple proxy posterior entropy
            double Hpost = 0.5*entropyBern(0.95) + 0.5*entropyBern(0.05);
            double w = clampd(1.0 - fabs(p0-0.5)*2.0, 0.0, 1.0);
            ig += w * max(0.0, H0 - Hpost);
        }
    }
    return ig;
}

static void applyObs(Grid& belief, const Grid& truth, Cell pose, const SensorParams& sp){
    for(int dy=-sp.r; dy<=sp.r; dy++){
        for(int dx=-sp.r; dx<=sp.r; dx++){
            int x=pose.first+dx, y=pose.second+dy;
            if(!belief.inb(x,y)) continue;
            bool isOcc = truth.isOcc(x,y,0.5);
            belief.setv(x,y, isOcc ? 0.95 : 0.05);
        }
    }
}

static bool pickGoalBudgeted(
    const Grid& belief,
    const vector<Cell>& frontiers,
    Cell cur, Cell home,
    const Budget& budget,
    const RobotParams& rp,
    const SensorParams& sp,
    double lam,
    Cell& bestGoal,
    vector<Cell>& bestPath
){
    double bestU = -1e18;
    bool found=false;
    // subsample if huge
    vector<Cell> cand=frontiers;
    if((int)cand.size()>80){
        std::mt19937 rng(7);
        shuffle(cand.begin(), cand.end(), rng);
        cand.resize(80);
    }

    for(auto g: cand){
        auto path = astar(belief, cur, g, 0.65);
        if(path.empty()) continue;
        int steps = max(0, (int)path.size()-1);
        double Ego = steps*(rp.E_step + rp.E_base);
        double Tgo = steps*rp.T_step;

        auto back = astar(belief, g, home, 0.65);
        if(back.empty()) continue;
        int bsteps = max(0, (int)back.size()-1);
        double Eback = bsteps*(rp.E_step + rp.E_base);
        double Tback = bsteps*rp.T_step;

        if(budget.E - (Ego+Eback) < rp.E_reserve) continue;
        if(budget.T - (Tgo+Tback) < 0) continue;

        double ig = expectedIG(belief, g, sp);
        double U = ig - lam*steps;
        if(U > bestU){
            bestU=U; bestGoal=g; bestPath=path; found=true;
        }
    }
    return found;
}

int main(){
    int W=35, H=25;
    Grid belief(W,H), truth(W,H);

    // truth: mostly free
    for(int y=0;y<H;y++) for(int x=0;x<W;x++) truth.setv(x,y,0.05);

    // borders
    for(int x=0;x<W;x++){ truth.setv(x,0,0.95); truth.setv(x,H-1,0.95); }
    for(int y=0;y<H;y++){ truth.setv(0,y,0.95); truth.setv(W-1,y,0.95); }

    // random rectangles
    std::mt19937 rng(3);
    auto rint=[&](int a,int b){ std::uniform_int_distribution<int> d(a,b); return d(rng); };
    for(int k=0;k<8;k++){
        int x0=rint(3,W-10), y0=rint(3,H-8);
        int ww=rint(3,7), hh=rint(2,5);
        for(int y=y0;y<min(H-1,y0+hh);y++)
            for(int x=x0;x<min(W-1,x0+ww);x++)
                truth.setv(x,y,0.95);
    }

    Cell home={2,2}, cur=home;

    RobotParams rp;
    SensorParams sp;
    Budget bud{160.0, 140.0};

    applyObs(belief, truth, cur, sp);

    int stepsDone=0, goals=0;

    while(true){
        auto fr = detectFrontiers(belief);
        if(fr.empty()){
            cout << "No frontiers left.\n"; break;
        }
        Cell goal; vector<Cell> path;
        bool ok = pickGoalBudgeted(belief, fr, cur, home, bud, rp, sp, 0.35, goal, path);
        if(!ok){
            cout << "No feasible frontier under budget. Returning.\n";
            break;
        }
        for(size_t i=1;i<path.size();i++){
            bud.E -= (rp.E_step + rp.E_base);
            bud.T -= rp.T_step;
            stepsDone++;
            cur = path[i];
            applyObs(belief, truth, cur, sp);
            if(bud.E < rp.E_reserve || bud.T <= 0) break;
        }
        goals++;
        if(bud.E < rp.E_reserve || bud.T <= 0) break;
        if(goals >= 40) { cout << "Stop: demo goal limit.\n"; break; }
    }

    auto back = astar(belief, cur, home, 0.65);
    if(!back.empty()){
        for(size_t i=1;i<back.size();i++){
            bud.E -= (rp.E_step + rp.E_base);
            bud.T -= rp.T_step;
            cur = back[i];
        }
    }
    cout << "Final pose: ("<<cur.first<<","<<cur.second<<") steps="<<stepsDone<<" goals="<<goals<<"\n";
    cout << "Remaining budget: E="<<bud.E<<" T="<<bud.T<<"\n";
    return 0;
}
