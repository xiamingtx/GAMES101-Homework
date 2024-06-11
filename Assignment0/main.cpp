#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

#define PI 3.1415926f
#define DEG2RAD (PI / 180.0f)

int main()
{
    // Define Point (2, 1) in the Homogenous Coordinates
    Eigen::Vector3f P(2, 1, 1);
    // Define the Transformation Matrix
    // [[cos theta, -sin theta, 1],
    //  [sin theta,  cos theta, 2],
    //  [    0,         0,      1]]
    Eigen::Matrix3f M;
    float theta = 45.0f * DEG2RAD;
    M << cos(theta), -sin(theta), 1, sin(theta), cos(theta), 2, 0, 0, 1;

    Eigen::Vector3f P2 = M * P;
    std::cout << P2 << std::endl;
    return 0;
}