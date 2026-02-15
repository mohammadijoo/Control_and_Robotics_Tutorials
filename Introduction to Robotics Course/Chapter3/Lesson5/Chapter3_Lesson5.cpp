#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>

bool dominates(const Eigen::VectorXd& ci, const Eigen::VectorXd& cj){
    bool ge_all = (ci.array() >= cj.array()).all();
    bool g_any  = (ci.array() >  cj.array()).any();
    return ge_all && g_any;
}

int main(){
    using Eigen::VectorXd;
    using Eigen::MatrixXd;

    std::vector<std::string> classes = {
        "IndustrialArm","MobileBase","MobileManipulator","Humanoid"
    };

    VectorXd r(4);
    r << 0.7, 0.8, 0.5, 0.9;

    MatrixXd C(4,4);
    C << 0.95,0.20,0.90,0.60,
         0.30,0.90,0.40,0.70,
         0.80,0.85,0.70,0.85,
         0.75,0.70,0.55,0.90;

    // Feasible set
    std::vector<int> feasible;
    for(int i=0;i<C.rows();++i){
        if( (C.row(i).array() >= r.transpose().array()).all() )
            feasible.push_back(i);
    }

    std::cout << "Feasible: ";
    for(int i: feasible) std::cout << classes[i] << " ";
    std::cout << std::endl;

    // Pareto set
    std::vector<int> pareto;
    for(int i: feasible){
        bool dom = false;
        for(int j: feasible){
            if(i==j) continue;
            if(dominates(C.row(j), C.row(i))) { dom = true; break; }
        }
        if(!dom) pareto.push_back(i);
    }

    std::cout << "Pareto: ";
    for(int i: pareto) std::cout << classes[i] << " ";
    std::cout << std::endl;
}
      