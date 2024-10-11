#pragma once

#include <cassert>
#include <cmath>
#include <vector>

class CubicSpline
{
public:
    CubicSpline(const std::vector<double>& bx, const std::vector<double>& by);
    ~CubicSpline(){};

    // Function to evaluate the spline at a given xVal
    double evaluate(double xVal);

    // Function to evaluate the derivative (velocity) of the spline at a given xVal
    double derivative(double xVal);

    // Generate trajectory points with distance ds
    std::vector<std::pair<double, double>> generateTrajectory(double ds);

private:
    std::vector<double> x_, y_;          // Input points
    std::vector<double> a_, b_, c_, d_;  // Spline coefficients
};