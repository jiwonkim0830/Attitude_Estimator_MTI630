#pragma once

#include <iostream>
#include <vector>
#include <mutex>
#include <Eigen/Dense>



class MahonyFilter
{
public:
    MahonyFilter();
private:
    double Kp, Ki, Ka, Km;

};