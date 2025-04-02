#include "BSpline.h"

BSpline::BSpline()
{
    degree_ = 3;
}

std::vector<Eigen::Vector2d> BSpline::interpolate(const std::vector<Eigen::Vector2d>& points, double ds)
{
    if (points.size() <= degree_)
    {
        throw std::invalid_argument("Number of control points must be greater than degree.");
    }

    std::vector<double> knots = generateKnotVector(points.size());
    std::vector<Eigen::Vector2d> splinePoints;

    double t = knots[degree_];  // Start from the first valid knot
    double totalArcLength = 0.0;
    Eigen::Vector2d prevPoint = points.front();

    double baselineLength = computeBaselineDistance(points);
    if (baselineLength == 0.0)
    {
        throw std::runtime_error("Baseline length is zero, invalid input points.");
    }

    double step = ds / (20 * baselineLength);
    step = std::max(step, 1e-5);  // Prevent step from being too small

    while (t <= knots[knots.size() - degree_ - 1])  // Traverse valid t-range
    {
        Eigen::Vector2d p(0.0, 0.0);

        for (size_t i = 0; i < points.size(); ++i)
        {
            double basisValue = B(i, degree_, t, knots);
            p += basisValue * points[i];
        }

        double dist = computeDistance(prevPoint, p);
        if (dist >= ds)
        {
            splinePoints.push_back(p);
            prevPoint = p;
        }

        t += step;
    }

    return splinePoints;
}

double BSpline::B(int i, int k, double t, const std::vector<double>& knots)
{
    if (k == 0)
    {
        return (t >= knots[i] && t <= knots[i + 1]) ? 1.0 : 0.0;
    }

    double left = 0.0, right = 0.0;
    
    double denominator_left = knots[i + k] - knots[i];
    if (denominator_left > 1e-9)
    {
        left = (t - knots[i]) / denominator_left * B(i, k - 1, t, knots);
    }

    double denominator_right = knots[i + k + 1] - knots[i + 1];
    if (denominator_right > 1e-9)
    {
        right = (knots[i + k + 1] - t) / denominator_right * B(i + 1, k - 1, t, knots);
    }

    return left + right;
}

std::vector<double> BSpline::generateKnotVector(int numPoints)
{
    int numKnots = numPoints + degree_ + 1;
    std::vector<double> knots(numKnots);

    for (int i = 0; i <= degree_; i++)
    {
        knots[i] = 0.0;
    }
    for (int i = degree_ + 1; i < numKnots - degree_ - 1; i++)
    {
        knots[i] = static_cast<double>(i - degree_) / (numPoints - degree_);
    }
    for (int i = numKnots - degree_ - 1; i < numKnots; i++)
    {
        knots[i] = 1.0;
    }

    return knots;
}

double BSpline::computeDistance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2)
{
    return (p2 - p1).norm();
}

double BSpline::computeBaselineDistance(const std::vector<Eigen::Vector2d>& points)
{
    double distance = 0;
    for (size_t i = 0; i < points.size() - 1; ++i)
    {
        distance += computeDistance(points[i], points[i + 1]);
    }
    return distance;
}
