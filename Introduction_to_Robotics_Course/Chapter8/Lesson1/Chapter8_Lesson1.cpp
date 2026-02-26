#include <iostream>
#include <vector>
#include <numeric>
#include <Eigen/Dense>

double movingAverage(const std::vector<double>& y, int k, int m){
    double s = 0.0;
    for(int i = 0; i < m; ++i){
        s += y[k - i];
    }
    return s / m;
}

int main(){
    // Example LS: y = Hx + v
    Eigen::MatrixXd H(3,2);
    H << 1, 0,
         1, 1,
         0, 1;
    Eigen::VectorXd y(3);
    y << 1.1, 2.0, 0.9;

    Eigen::VectorXd x_hat = (H.transpose()*H).inverse() * H.transpose() * y;
    std::cout << "LS estimate x_hat =\\n" << x_hat << std::endl;
    return 0;
}
