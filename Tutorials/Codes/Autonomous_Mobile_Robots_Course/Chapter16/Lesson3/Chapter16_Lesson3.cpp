// Chapter16_Lesson3.cpp
// Socially aware local velocity selection (sampling-based) for a point robot.
// Output: CSV trajectories for robot and humans.
// Build (example): g++ -O2 -std=c++17 Chapter16_Lesson3.cpp -o run
// Run: ./run  (produces out_robot.csv and out_humans.csv)

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include <limits>

struct Vec2 {
    double x{0}, y{0};
    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& o) const { return {x+o.x, y+o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x-o.x, y-o.y}; }
    Vec2 operator*(double s) const { return {x*s, y*s}; }
};

static inline double dot(const Vec2& a, const Vec2& b){ return a.x*b.x + a.y*b.y; }
static inline double norm2(const Vec2& a){ return dot(a,a); }
static inline double norm(const Vec2& a){ return std::sqrt(norm2(a)); }

struct Human { Vec2 p; Vec2 v; };

static inline Vec2 rotT(const Vec2& r, double th){
    double c = std::cos(th), s = std::sin(th);
    return { c*r.x + s*r.y, -s*r.x + c*r.y };
}

double anisotropic_gaussian_cost(const Vec2& x, const std::vector<Human>& humans,
                                 double sigma_front=1.2, double sigma_side=0.6, double sigma_back=0.8){
    double cost = 0.0;
    for(const auto& h: humans){
        double spd = norm(h.v);
        double th = (spd > 1e-6) ? std::atan2(h.v.y, h.v.x) : 0.0;
        Vec2 r = x - h.p;
        Vec2 rh = rotT(r, th);
        double sx = (rh.x >= 0.0) ? sigma_front : sigma_back;
        double sy = sigma_side;
        double q = (rh.x*rh.x)/(sx*sx) + (rh.y*rh.y)/(sy*sy);
        cost += std::exp(-0.5*q);
    }
    return cost;
}

bool collision_within_tau(const Vec2& v_robot, const Vec2& p_robot, const std::vector<Human>& humans,
                          double R=0.55, double tau=3.0){
    for(const auto& h: humans){
        Vec2 p = h.p - p_robot;
        Vec2 vrel = v_robot - h.v;
        double vv = norm2(vrel);
        if(vv < 1e-12){
            if(norm(p) < R) return true;
            continue;
        }
        double tstar = -dot(p, vrel)/vv;
        if(tstar < 0.0) tstar = 0.0;
        if(tstar > tau) tstar = tau;
        Vec2 closest = p - vrel*tstar;
        if(norm(closest) < R) return true;
    }
    return false;
}

std::vector<Vec2> sample_velocities(const Vec2& v_pref, double v_max=1.2, int n_speed=9, int n_angle=21, double spread_deg=80.0){
    double spref = norm(v_pref);
    double th0 = (spref > 1e-8) ? std::atan2(v_pref.y, v_pref.x) : 0.0;
    double spread = spread_deg * M_PI / 180.0;

    std::vector<Vec2> V;
    for(int i=0;i<n_speed;i++){
        double s = (double)i/(n_speed-1) * v_max;
        for(int j=0;j<n_angle;j++){
            double a = -spread + 2.0*spread*(double)j/(n_angle-1);
            double th = th0 + a;
            V.emplace_back(s*std::cos(th), s*std::sin(th));
        }
    }
    return V;
}

Vec2 choose_velocity(const Vec2& p_robot, const Vec2& v_pref, const std::vector<Human>& humans,
                     double v_max=1.2, double w_track=1.0, double w_social=1.5, double w_clear=2.0,
                     double R=0.55, double tau=3.0){
    double bestJ = std::numeric_limits<double>::infinity();
    Vec2 bestV{0,0};
    const double dt_eval = 0.6;
    const double eps = 1e-3;

    for(const auto& v: sample_velocities(v_pref, v_max)){
        if(collision_within_tau(v, p_robot, humans, R, tau)) continue;

        Vec2 p_next = p_robot + v*dt_eval;
        double C = anisotropic_gaussian_cost(p_next, humans);
        double clear = 0.0;
        for(const auto& h: humans){
            double d = norm(p_next - h.p);
            clear += 1.0/(d + eps);
        }

        Vec2 dv = Vec2{v.x - v_pref.x, v.y - v_pref.y};
        double J = w_track*norm2(dv) + w_social*C + w_clear*clear;

        if(J < bestJ){
            bestJ = J;
            bestV = v;
        }
    }
    return bestV;
}

int main(){
    double dt = 0.1;
    double T = 26.0;
    int steps = (int)std::round(T/dt);

    Vec2 p{0.0, 0.0};
    Vec2 goal{10.0, 0.0};
    double v_max = 1.2;

    std::vector<Human> humans = {
        {Vec2{4.5,  1.2}, Vec2{ 0.35, -0.05}},
        {Vec2{4.0, -1.4}, Vec2{ 0.35,  0.08}},
        {Vec2{7.0,  0.3}, Vec2{-0.25,  0.02}}
    };

    std::ofstream fR("out_robot.csv");
    std::ofstream fH("out_humans.csv");
    fR << "t,x,y\n";
    fH << "t,id,x,y\n";

    for(int k=0;k<steps;k++){
        double t = k*dt;

        fR << t << "," << p.x << "," << p.y << "\n";
        for(size_t i=0;i<humans.size();i++){
            fH << t << "," << (i+1) << "," << humans[i].p.x << "," << humans[i].p.y << "\n";
        }

        for(auto& h: humans){
            h.p = h.p + h.v*dt;
        }

        Vec2 e = goal - p;
        double dist = norm(e);
        if(dist < 0.15) break;
        Vec2 v_pref = e*(1.0/(dist + 1e-12));
        double s = std::min(v_max, 0.8*dist);
        v_pref = v_pref*s;

        Vec2 v_cmd = choose_velocity(p, v_pref, humans, v_max, 1.0, 1.5, 2.0, 0.55, 3.0);
        p = p + v_cmd*dt;
    }

    std::cout << "Wrote out_robot.csv and out_humans.csv\n";
    return 0;
}
