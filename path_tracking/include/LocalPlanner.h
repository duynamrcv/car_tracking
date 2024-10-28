#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include "CarModel.h"

class LocalPlanner
{
public:
    LocalPlanner(const std::vector<Pose>& globalPath, const Pose& pose, const int order);
    ~LocalPlanner();

    void updateVehiclePose(const Pose& pose);
    void findClosestWaypointAhead();
    std::vector<Pose> getLocalPathAhead(const int& numPoseAhead);
    std::vector<Pose> getGlobalPathAhead(const int& numPoseAhead);
    std::vector<Pose> genLocalPathInter(const Pose& vehiclePose, const int& numPoseAhead,
                                            const int& numPoints, const double& step);
    std::vector<Pose> genLocalPathInterEqual(const Pose& vehiclePose, const int& numPoseAhead,
                                                 const int& numPoints, const double& step);
    std::vector<Pose> convertLocalToGlobal(const std::vector<Pose>& localTrajectory);

private:
    Eigen::VectorXd fitPolynomial(const std::vector<Eigen::Vector2d>& waypoints) const;
    static std::vector<Pose> generatePointsWithHeading(const Eigen::VectorXd& coefficient,
                                                           const Pose& currentPose,
                                                           const int& numPoints,
                                                           const double& step);
    static double normalizeAngle(double angle);
    static double evaluatePolynomial(const Eigen::VectorXd& coefficients, const double& x);
    static double evaluateDerivative(const Eigen::VectorXd& coefficients, const double& x);

    std::vector<Pose> globalPath_;
    Pose vehiclePose_;
    size_t vehicleIndex_;
    int order_;
};