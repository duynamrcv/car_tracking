#pragma once

#include "Spline.h"
#include "Spline1D.h"

class CubicSpline : public Spline
{
public:
    CubicSpline()  = default;
    ~CubicSpline() = default;

    std::vector<Eigen::Vector2d> interpolate(const std::vector<Eigen::Vector2d>& points, double ds);

private:
    std::vector<double> computeS(const std::vector<Eigen::Vector2d>& points);
    Eigen::Vector2d computePosition(Spline1D& sx, Spline1D& sy, double sVal);
};