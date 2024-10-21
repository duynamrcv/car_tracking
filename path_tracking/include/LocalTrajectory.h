#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include "CarModel.h"

class LocalTrajectory
{
public:
    LocalTrajectory(const std::vector<WayPoint>& globalPath, const Pose& pose, const int order);
    ~LocalTrajectory();

    void updateVehiclePose(const Pose& pose);
    void findClosestWaypointAhead();
    std::vector<WayPoint> getLocalPathAhead(const int& numPoints);
    std::vector<WayPoint> getGlobalPathAhead(const int& numPoints);

private:
    std::vector<WayPoint> convertLocalToGlobal(const std::vector<WayPoint>& localTrajectory) const;
    Eigen::VectorXd fitPolynomial(const std::vector<Eigen::Vector2d>& waypoints) const;
    static std::vector<WayPoint> generatePointsWithHeading(const Eigen::VectorXd& coefficient,
                                                           const Pose& currentPose,
                                                           const int& numPoints,
                                                           const double& step);
    static double normalizeAngle(double angle);
    static double evaluatePolynomial(const Eigen::VectorXd& coefficients, const double& x);
    static double evaluateDerivative(const Eigen::VectorXd& coefficients, const double& x);

    std::vector<WayPoint> globalPath_;
    Pose vehiclePose_;
    size_t vehicleIndex_;
    int order_;
};