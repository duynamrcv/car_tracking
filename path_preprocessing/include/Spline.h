#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include <cmath>

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
    std::vector<Eigen::Vector2d> interpolate(const std::vector<Eigen::Vector2d>& points, double ds);
};