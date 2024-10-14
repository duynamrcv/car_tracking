#include "Spline.h"

void SplineContext::setStrategy(Spline* strategy)
{
    this->strategy = strategy;
}

std::vector<Eigen::Vector2d> SplineContext::interpolate(const std::vector<Eigen::Vector2d>& points,
                                                        double ds)
{
    std::vector<Eigen::Vector2d> trajectory = strategy->interpolate(points, ds);
    std::cout << "Trajectory length: " << trajectory.size() << std::endl;
    return trajectory;
}