#define _USE_MATH_DEFINES
#include "../include/AnalyticalSolver.hpp"
#include <cmath>
#include <iostream>
 
static const double g = 9.81;
 
Ballistics AnalyticalSolver::solve(float altitude, float speed, const AmmoParams& ammo)
{
    const float m  = ammo.mass;
    const float d  = ammo.drag;
    const float l  = ammo.lift;
    const float v  = speed;
    const float zd = altitude;
 
    double a = d * g * m - 2 * std::pow(d, 2) * l * v;
    double b = -3 * g * std::pow(m, 2) + 3 * d * l * m * v;
    double c = 6 * std::pow(m, 2) * zd;
 
    double p = -(std::pow(b, 2) / (3 * std::pow(a, 2)));
    double q = (2 * std::pow(b, 3)) / (27 * std::pow(a, 3)) + c / a;
 
    double acosArg = (3 * q) / (2 * p) * std::sqrt(-3.0 / p);
    if (acosArg < -1.0 || acosArg > 1.0)
    {
        std::cerr << "Model invalid for altitude\n";
        return { -1.0, -1.0 };
    }
 
    double phi = std::acos(acosArg);
    double time = 2 * std::sqrt(-p / 3.0) * std::cos((phi + 4 * M_PI) / 3.0) - b / (3 * a);
    if (time <= 0)
    {
        std::cerr << "Time invalid\n";
        return { -1.0, -1.0 };
    }
 
    double h = v * time
        - std::pow(time, 2) * d * v / (2 * m)
        + std::pow(time, 3) * (6 * d * g * l * m - 6 * std::pow(d, 2) * (std::pow(l, 2) - 1) * v) / (36 * std::pow(m, 2))
        + std::pow(time, 4) * (-6 * std::pow(d, 2) * g * l * (1 + std::pow(l, 2) + std::pow(l, 4)) * m
            + 3 * std::pow(d, 3) * std::pow(l, 2) * (1 + std::pow(l, 2)) * v
            + 6 * std::pow(d, 3) * std::pow(l, 4) * (1 + std::pow(l, 2)) * v)
            / (36 * std::pow(1 + std::pow(l, 2), 2) * std::pow(m, 3))
        + std::pow(time, 5) * (3 * std::pow(d, 3) * g * std::pow(l, 3) * m
            - 3 * std::pow(d, 4) * std::pow(l, 2) * (1 + std::pow(l, 2)) * v)
            / (36 * (1 + std::pow(l, 2)) * std::pow(m, 4));
 
    if (h <= 0)
    {
        std::cerr << "Horizontal length invalid\n";
        return { -1.0, -1.0 };
    }
 
    return { h, time };
}