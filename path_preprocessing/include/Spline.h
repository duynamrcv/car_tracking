#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include <cmath>

struct WayPoint
{
    double x;
    double y;
    double yaw;
    double v;
    double steer;
};

class Spline
{
public:
    virtual std::vector<Eigen::Vector2d> interpolate(const std::vector<Eigen::Vector2d>& points, double ds) = 0;
};

class SplineContext
{
private:
    Spline* strategy;
public:
    void setStrategy(Spline* strategy);
    std::vector<WayPoint> computeYaw(const std::vector<Eigen::Vector2d>& points);
    std::vector<WayPoint> interpolate(const std::vector<Eigen::Vector2d>& points, double ds);
};