#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){
    Eigen::Vector3f p(2.0,1.0,1.0); 
    Eigen::Matrix3f transform ;
    double theta = M_PI/4.0;
    transform << cos(theta), -sin(theta), 1,
                 sin(theta), cos(theta), 2,
                 0, 0, 1;
    p = transform*p;
    std::cout<<p<<"\n";
    
    return 0;

}