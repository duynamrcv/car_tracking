#pragma once

#include "Spline.h"

class Spline1D
{
public:
    Spline1D(const std::vector<double>& x_, const std::vector<double>& y_);
    double compute(double t);
    double computeD(double t);
    double computeDD(double t);

private:
    int searchIndex(double t);
    Eigen::MatrixXd computeA(const std::vector<double> h);
    Eigen::MatrixXd computeB(const std::vector<double> h);

    std::vector<double> x_, y_;          // Input points
    std::vector<double> a_, b_, c_, d_;  // Spline coefficients
    int nx_;
};