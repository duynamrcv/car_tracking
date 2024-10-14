#pragma once

#include "Spline.h"

class BSpline : public Spline
{
public:
    BSpline();
    ~BSpline();

    virtual std::vector<Eigen::Vector2d> interpolate(const std::vector<Eigen::Vector2d>& points, double ds);
private:
    double B(int i, int k, double t, const std::vector<double>& knots);
    std::vector<double> generateKnotVector(int numPoints);
    double computeDistance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);
    double computeBaselineDistance(const std::vector<Eigen::Vector2d>& points);

    int degree_;
};