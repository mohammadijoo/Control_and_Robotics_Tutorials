#include <vector>
#include <iostream>
#include <Eigen/Dense>

struct Agent {
    Eigen::Vector2d p;
    Eigen::Vector2d v;
    Eigen::Vector2d v_pref;
    double r;
    double max_speed;
};

double timeToCollision(const Eigen::Vector2d& p_ij,
                       const Eigen::Vector2d& v_ij,
                       double R, double T_h)
{
    double a = v_ij.squaredNorm();
    double c = p_ij.squaredNorm() - R*R;
    if (c <= 0.0) return 0.0;
    if (a == 0.0) return std::numeric_limits<double>::infinity();
    double b = 2.0 * p_ij.dot(v_ij);
    double disc = b*b - 4.0*a*c;
    if (disc < 0.0) return std::numeric_limits<double>::infinity();
    double t_min = (-b - std::sqrt(disc)) / (2.0*a);
    if (t_min < 0.0 || t_min > T_h)
        return std::numeric_limits<double>::infinity();
    return t_min;
}

Eigen::Vector2d normalize(const Eigen::Vector2d& v)
{
    double n = v.norm();
    if (n == 0.0) return v;
    return v / n;
}

struct HalfPlane {
    Eigen::Vector2d n;
    Eigen::Vector2d p0; // (v - p0) . n >= 0
};

std::vector<HalfPlane> orcaConstraintsForAgent(
    int i,
    const std::vector<Agent>& agents,
    double T_h,
    double delta)
{
    const Agent& ai = agents[i];
    std::vector<HalfPlane> cons;

    for (int j = 0; j < (int)agents.size(); ++j) {
        if (j == i) continue;
        const Agent& aj = agents[j];
        Eigen::Vector2d p_ij = aj.p - ai.p;
        Eigen::Vector2d v_ij = ai.v - aj.v;
        double R = ai.r + aj.r;
        double ttc = timeToCollision(p_ij, v_ij, R, T_h);
        if (ttc == std::numeric_limits<double>::infinity())
            continue;

        Eigen::Vector2d p_coll = p_ij + ttc * v_ij;
        Eigen::Vector2d n_ij = normalize(p_coll);
        Eigen::Vector2d u_ij = (R * n_ij - p_coll) / std::max(ttc, delta);
        Eigen::Vector2d p0 = ai.v + 0.5 * u_ij;
        cons.push_back({n_ij, p0});
    }
    return cons;
}

Eigen::Vector2d projectVelocity(
    const Eigen::Vector2d& v_pref,
    const std::vector<HalfPlane>& cons,
    double max_speed)
{
    Eigen::Vector2d v = v_pref;
    double nrm = v.norm();
    if (nrm > max_speed) v *= max_speed / nrm;

    for (const auto& hp : cons) {
        double val = (v - hp.p0).dot(hp.n);
        if (val < 0.0) {
            v = v + (hp.p0 - v).dot(hp.n) * hp.n;
            nrm = v.norm();
            if (nrm > max_speed) v *= max_speed / nrm;
        }
    }
    return v;
}

void stepORCA(std::vector<Agent>& agents,
              double T_h, double delta)
{
    std::vector<Eigen::Vector2d> new_v(agents.size());

    for (int i = 0; i < (int)agents.size(); ++i) {
        auto cons = orcaConstraintsForAgent(i, agents, T_h, delta);
        new_v[i] = projectVelocity(agents[i].v_pref, cons, agents[i].max_speed);
    }

    for (int i = 0; i < (int)agents.size(); ++i) {
        agents[i].v = new_v[i];
        agents[i].p += delta * agents[i].v;
    }
}
      
