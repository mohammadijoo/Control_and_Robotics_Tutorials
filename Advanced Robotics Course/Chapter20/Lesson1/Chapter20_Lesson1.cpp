#include <iostream>
#include <vector>
#include <string>

struct Project {
    std::string name;
    double I_norm;
    double A_norm;
    double D_norm;
    double R_norm;
};

struct Weights {
    double lambda_I;
    double lambda_A;
    double lambda_D;
    double lambda_R;
};

double utility(const Project& p, const Weights& w) {
    return w.lambda_I * p.I_norm
         + w.lambda_A * p.A_norm
         - w.lambda_D * p.D_norm
         - w.lambda_R * p.R_norm;
}

int main() {
    std::vector<Project> projects = {
        {"Kinodynamic RRT* with risk-sensitive cost", 0.8, 0.9, 0.7, 0.6},
        {"Task and motion planning for pick-place",    0.7, 0.8, 0.5, 0.4},
        {"Swarm coverage with formal specs",           0.9, 0.7, 0.9, 0.8}
    };

    Weights w {0.4, 0.3, 0.2, 0.1};

    double bestScore = -1e9;
    int bestIndex = -1;

    for (std::size_t i = 0; i < projects.size(); ++i) {
        double score = utility(projects[i], w);
        if (score > bestScore) {
            bestScore = score;
            bestIndex = static_cast<int>(i);
        }
    }

    if (bestIndex >= 0) {
        std::cout << "Best project: " << projects[bestIndex].name
                  << " with score " << bestScore << std::endl;
    }
    return 0;
}
      
