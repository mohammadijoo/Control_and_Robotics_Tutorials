
#include 
#include 

std::string classifySystem(double cT, double iEU, int rR,
                           double th_c=0.3, double th_i=0.2, int th_r=2){
    if(cT < th_c && iEU < th_i && rR == 1) return "Automation";
    if(cT >= th_c && iEU >= th_i && rR >= th_r) return "Robot";
    return "Mechatronic System";
}

int main(){
    std::cout << "CNC line -> " << classifySystem(0.1, 0.05, 1) << "\n";
    std::cout << "Active suspension -> " << classifySystem(0.4, 0.15, 2) << "\n";
    std::cout << "Mobile robot -> " << classifySystem(0.7, 0.6, 5) << "\n";
    return 0;
}
      