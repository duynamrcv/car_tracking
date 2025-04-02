#include "BSpline.h"

BSpline::BSpline()
{
    degree_ = 3;
}

std::vector<Eigen::Vector2d> BSpline::interpolate(const std::vector<Eigen::Vector2d>& points,
                                                  double ds)
{
    // Generate knot vector
    std::vector<double> knots = generateKnotVector(points.size());

    // Initialize the output points
    std::vector<Eigen::Vector2d> splinePoints;

    double t                  = 0.0;  // Parameter to traverse the spline
    double totalArcLength     = 0.0;
    Eigen::Vector2d prevPoint = points[points.size() - 1];

    // Compute baseline distance and step size
    double baselineLength = computeBaselineDistance(points);
    std::cout << "step: " << baselineLength << std::endl;
    double step           = ds / (20 * baselineLength);

    // Loop to generate the spline based on the distance 'ds'
    while (t < 1.0)
    {
        Eigen::Vector2d p(0.0, 0.0);

        // Compute spline point for parameter t
        for (int i = 0; i < points.size(); ++i)
        {
            double basisValue = B(i, degree_, t, knots);
            p += basisValue * points[i];
        }

        // Calculate the distance from the previous point
        double dist = computeDistance(prevPoint, p);

        // If the distance exceeds the specified 'ds', store the point
        if (dist >= ds)
        {
            splinePoints.push_back(p);
            prevPoint = p;  // Update the previous point to the current point
        }

        // Increment t following step size
        t += step;
    }

    return splinePoints;
}

// Function to calculate the B-spline basis function
double BSpline::B(int i, int k, double t, const std::vector<double>& knots)
{
    if (k == 0)
    {
        return (t >= knots[i] && t < knots[i + 1]) ? 1.0 : 0.0;
    }
    else
    {
        double left  = 0.0;
        double right = 0.0;

        double denominator_left = knots[i + k] - knots[i];
        if (denominator_left != 0)
        {
            left = (t - knots[i]) / denominator_left * B(i, k - 1, t, knots);
        }

        double denominator_right = knots[i + k + 1] - knots[i + 1];
        if (denominator_right != 0)
        {
            right = (knots[i + k + 1] - t) / denominator_right * B(i + 1, k - 1, t, knots);
        }

        return left + right;
    }
}

// Function to generate knot vector
std::vector<double> BSpline::generateKnotVector(int numPoints)
{
    int n = numPoints + degree_ + 1;
    std::vector<double> knots(n);

    for (int i = 0; i <= degree_; i++)
    {
        knots[i] = 0.0;
    }
    for (int i = degree_ + 1; i < n - degree_ - 1; i++)
    {
        knots[i] = static_cast<double>(i - degree_) / (n - 2 * degree_);
    }
    for (int i = n - degree_ - 1; i < n; i++)
    {
        knots[i] = 1.0;
    }

    return knots;
}

// Function to compute Euclidean distance between two points
double BSpline::computeDistance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2)
{
    return std::sqrt(std::pow(p2.x() - p1.x(), 2) + std::pow(p2.y() - p1.y(), 2));
}

// Function to compute Euclidean distance of baseline points
double BSpline::computeBaselineDistance(const std::vector<Eigen::Vector2d>& points)
{
    double distance = 0;
    for (int i = 0; i < points.size() - 1; ++i)
    {
        distance += computeDistance(points[i], points[i + 1]);
    }
    return distance;
}