#include "Spline.h"

void SplineContext::setStrategy(Spline* strategy)
{
    this->strategy = strategy;
}

std::vector<WayPoint> SplineContext::computeYaw(const std::vector<Eigen::Vector2d>& points)
{
    std::vector<WayPoint> trajectory;
    int length = points.size();
    for (int i = 0; i < length; i++)
    {
        WayPoint wp;
        wp.x = points[i].x();
        wp.y = points[i].y();
        if (i < length - 1)
        {
            double dx = points[i + 1].x() - points[i].x();
            double dy = points[i + 1].y() - points[i].y();
            wp.yaw    = atan2(dy, dx);
        }
        else
        {
            wp.yaw = trajectory.back().yaw;
        }
        wp.v     = 0.0;
        wp.steer = 0.0;
        trajectory.emplace_back(wp);
    }
    return trajectory;
}

std::vector<WayPoint> SplineContext::interpolate(const std::vector<Eigen::Vector2d>& points,
                                                 double ds)
{
    std::vector<Eigen::Vector2d> trajectory2D = strategy->interpolate(points, ds);
    std::vector<WayPoint> trajectory          = computeYaw(trajectory2D);
    std::cout << "Trajectory length: " << trajectory.size() << std::endl;
    return trajectory;
}