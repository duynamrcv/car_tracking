#include "CubicSpline.h"

CubicSpline::CubicSpline() {}

CubicSpline::~CubicSpline() {}

std::vector<Eigen::Vector2d> CubicSpline::interpolate(const std::vector<Eigen::Vector2d>& points,
                                                      double ds)
{
    std::vector<double> s = computeS(points);
    std::vector<double> x, y;
    for(int i = 0; i < points.size(); i++)
    {
        x.push_back(points[i].x());
        y.push_back(points[i].y());
    }

    Spline1D sx(computeS(points), x);
    Spline1D sy(computeS(points), y);
    double maxS = s.back();

    std::vector<Eigen::Vector2d> splinePoints;
    for (double sVal = 0; sVal < maxS; sVal += ds) {
        splinePoints.push_back(computePosition(sx, sy, sVal));
    }
    return splinePoints;
}

std::vector<double> CubicSpline::computeS(const std::vector<Eigen::Vector2d>& points)
{
    // compute the cumulative arc length
    std::vector<double> ds(points.size() - 1);
    std::vector<double> sVal = {0};
    for (size_t i = 1; i < points.size(); i++)
    {
        double dx = points[i].x() - points[i - 1].x();
        double dy = points[i].y() - points[i - 1].y();
        ds[i - 1] = hypot(dx, dy);
        sVal.push_back(sVal.back() + ds[i - 1]);
    }
    return sVal;
}

Eigen::Vector2d CubicSpline::computePosition(Spline1D& sx, Spline1D& sy, double sVal)
{
    double x = sx.compute(sVal);
    double y = sy.compute(sVal);
    return Eigen::Vector2d(x, y);
}