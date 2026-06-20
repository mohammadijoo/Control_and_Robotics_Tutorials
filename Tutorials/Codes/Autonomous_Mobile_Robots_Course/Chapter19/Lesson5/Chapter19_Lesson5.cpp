// Chapter19_Lesson5.cpp
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <numeric>
struct R{std::string s;int seed,succ,col;double t,pl,sp,clr,en,ate,rpe,cm,cp;};
std::vector<std::string> split(const std::string& x){std::stringstream ss(x); std::vector<std::string> v; std::string a; while(std::getline(ss,a,',')) v.push_back(a); return v;}
std::vector<R> readcsv(const std::string& p){std::ifstream f(p); std::string line; std::getline(f,line); std::vector<R> rs; while(std::getline(f,line)){auto t=split(line); if(t.size()<13) continue; rs.push_back({t[0],stoi(t[1]),stoi(t[2]),stoi(t[6]),stod(t[3]),stod(t[4]),stod(t[5]),stod(t[7]),stod(t[8]),stod(t[9]),stod(t[10]),stod(t[11]),stod(t[12])});} return rs;}
double mean(const std::vector<double>& x){return std::accumulate(x.begin(),x.end(),0.0)/x.size();}
double q(std::vector<double> x,double p){std::sort(x.begin(),x.end()); double k=(x.size()-1)*p; size_t i=floor(k), j=ceil(k); return i==j?x[i]:x[i]*(j-k)+x[j]*(k-i);} 
int main(int argc,char**argv){ if(argc<3){std::cerr<<"Usage: "<<argv[0]<<" baseline.csv candidate.csv\n"; return 1;} auto A=readcsv(argv[1]); auto B=readcsv(argv[2]); auto agg=[&](const std::vector<R>& rs){std::vector<double> succ,spl,col,t,ate,rpe,clr,eper,cm,cp; for(auto&r:rs){succ.push_back(r.succ); spl.push_back(r.succ? r.sp/std::max(r.pl,1e-9):0.0); col.push_back(r.col>0); t.push_back(r.t); ate.push_back(r.ate); rpe.push_back(r.rpe); clr.push_back(r.clr); eper.push_back(r.en/std::max(r.pl,1e-9)); cm.push_back(r.cm); cp.push_back(r.cp);} std::cout<<std::fixed<<std::setprecision(4)<<mean(succ)<<","<<mean(spl)<<","<<mean(col)<<","<<mean(t)<<","<<mean(ate)<<","<<mean(rpe)<<","<<q(clr,0.1)<<","<<mean(eper)<<","<<mean(cm)<<","<<mean(cp)<<"\n";};
std::cout<<"success,SPL,collision,time,ATE,RPE,clrP10,Eperm,CPUmean,CPUp95\n"; agg(A); agg(B); }
