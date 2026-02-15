
#include <iostream>
#include <unordered_map>
#include <vector>

int main() {
    std::unordered_map<char,int> dof = {
        {'R',1},{'P',1},{'H',1},{'C',2},{'U',2},{'S',3},{'E',3}
    };

    int n_links = 4; // including ground
    std::vector<char> joints = {'R','R','R'}; // a 3R serial arm

    int M = 6*(n_links-1);
    for(char j : joints) M -= (6 - dof[j]);

    std::cout << "Mobility = " << M << std::endl;
    return 0;
}
      