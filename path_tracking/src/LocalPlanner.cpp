#include "LocalPlanner.h"

LocalPlanner::LocalPlanner(const std::vector<WayPoint>& globalPath, const Pose& pose,
                           const int order)
{
    globalPath_   = globalPath;
    vehiclePose_  = pose;
    order_        = order;
    vehicleIndex_ = 0;
}
LocalPlanner::~LocalPlanner() {}

void LocalPlanner::updateVehiclePose(const Pose& pose)
{
    vehiclePose_ = pose;
}

void LocalPlanner::findClosestWaypointAhead()
{
    constexpr int searchWindow = 50;  // Limit the search to the next 50 points
    double minDistance         = std::numeric_limits<double>::max();

    for (size_t i = vehicleIndex_; i < std::min(vehicleIndex_ + searchWindow, globalPath_.size());
         ++i)
    {
        double dx = globalPath_[i].x - vehiclePose_.x;
        double dy = globalPath_[i].y - vehiclePose_.y;

        // Ensure the waypoint is ahead of the vehicle
        double dis = sqrt(dx * dx + dy * dy);
        if (dis < minDistance && dx * cos(vehiclePose_.yaw) + dy * sin(vehiclePose_.yaw) > 0)
        {
            minDistance   = dis;
            vehicleIndex_ = i;
        }
    }
}

std::vector<WayPoint> LocalPlanner::genLocalPathInter(const Pose& vehiclePose,
                                                      const int& numPoseAhead, const int& numPoints,
                                                      const double& step)
{
    // Update the vehicle's pose
    updateVehiclePose(vehiclePose);

    // Get the local path ahead of the vehicle using the specified number of poses
    std::vector<WayPoint> localPathAhead = getLocalPathAhead(numPoseAhead);
    if (localPathAhead.size() < numPoseAhead)
    {
        // TODO: add as end point
        std::vector<WayPoint> localTrajectory;
        for (int i = 0; i < numPoints; i++)
        {
            int index = vehicleIndex_ + i;
            if (index >= globalPath_.size()) index = globalPath_.size() - 1;
            localTrajectory.emplace_back(globalPath_[index]);
        }
        return localTrajectory;
    }
    else
    {
        // Convert the local path to (x, y) points to fit the polynomial
        std::vector<Eigen::Vector2d> waypoints;
        for (const auto& point : localPathAhead)
        {
            waypoints.emplace_back(Eigen::Vector2d(point.x, point.y));
        }

        // Fit a polynomial to the waypoints (5th order)
        const Eigen::VectorXd coefficients = fitPolynomial(waypoints);

        // Generate points with heading using the internally computed coefficients
        Pose pose;
        pose.x   = 0.0;
        pose.y   = 0.0;
        pose.yaw = 0.0;
        std::vector<WayPoint> pointsWithHeading =
            generatePointsWithHeading(coefficients, pose, numPoints, step);

        // Convert the generated local path back to global coordinates
        return convertLocalToGlobal(pointsWithHeading);
    }
}

std::vector<WayPoint> LocalPlanner::genLocalPathInterEqual(const Pose& vehiclePose,
                                                           const int& numPoseAhead,
                                                           const int& numPoints, const double& step)
{
    // FIXME: check carrefully to ensure the length of output
    double smallStep = 0.01;

    // number point dense static_cast
    int numPointsDense = static_cast<int>(numPoints * step / smallStep);
    auto allPoints     = genLocalPathInter(vehiclePose, numPoseAhead, numPointsDense, smallStep);

    // check and select the point from all_points with distance equal step
    std::vector<WayPoint> pointsWithHeading;
    pointsWithHeading.push_back(allPoints[0]);
    double distance = 0;
    for (int i = 1; i < allPoints.size(); i++)
    {
        double x1 = allPoints[i].x;
        double y1 = allPoints[i].y;
        double x0 = pointsWithHeading.back().x;
        double y0 = pointsWithHeading.back().y;
        distance  = sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));
        if (distance >= step)
        {
            pointsWithHeading.push_back(allPoints[i]);
            distance = 0;
        }
        if (allPoints.size() == numPoints) break;
    }
    return pointsWithHeading;
}

std::vector<WayPoint> LocalPlanner::getLocalPathAhead(const int& numPoseAhead)
{
    std::vector<WayPoint> localPath;  // Ego waypoint

    findClosestWaypointAhead();
    for (size_t i = vehicleIndex_; i < vehicleIndex_ + numPoseAhead && i < globalPath_.size(); ++i)
    {
        const double dx = globalPath_[i].x - vehiclePose_.x;
        const double dy = globalPath_[i].y - vehiclePose_.y;

        // Convert global coordinates to local coordinates
        WayPoint wp;
        wp.x     = dx * cos(-vehiclePose_.yaw) - dy * sin(-vehiclePose_.yaw);
        wp.y     = dx * sin(-vehiclePose_.yaw) + dy * cos(-vehiclePose_.yaw);
        wp.yaw   = normalizeAngle(globalPath_[i].yaw - vehiclePose_.yaw);
        wp.v     = globalPath_[i].v;
        wp.steer = globalPath_[i].steer;

        localPath.emplace_back(wp);
    }

    // If not enough points, add the last point repeatedly to match the required number of points
    // while (localPath.size() < static_cast<size_t>(numPoseAhead))
    // {
    //     localPath.emplace_back(localPath.back());
    // }

    return localPath;
}

std::vector<WayPoint> LocalPlanner::getGlobalPathAhead(const int& numPoseAhead)
{
    std::vector<WayPoint> globalPathAhead;

    findClosestWaypointAhead();
    for (size_t i = vehicleIndex_; i < vehicleIndex_ + numPoseAhead && i < globalPath_.size(); ++i)
    {
        globalPathAhead.emplace_back(globalPath_[i]);
    }

    // If not enough points, add the last point to match the required number of points
    while (globalPathAhead.size() < static_cast<size_t>(numPoseAhead))
    {
        // +1 because of ego-point
        globalPathAhead.emplace_back(globalPathAhead.back());
    }

    return globalPathAhead;
}

std::vector<WayPoint> LocalPlanner::convertLocalToGlobal(
    const std::vector<WayPoint>& localTrajectory)
{
    std::vector<WayPoint> globalTrajectory;
    for (const WayPoint& point : localTrajectory)
    {
        WayPoint wp;
        wp.x   = vehiclePose_.x + point.x * cos(vehiclePose_.yaw) - point.y * sin(vehiclePose_.yaw);
        wp.y   = vehiclePose_.y + point.x * sin(vehiclePose_.yaw) + point.y * cos(vehiclePose_.yaw);
        wp.yaw = normalizeAngle(point.yaw + vehiclePose_.yaw);
        wp.v   = point.v;
        wp.steer = point.steer;

        globalTrajectory.emplace_back(wp);
    }

    return globalTrajectory;
}

Eigen::VectorXd LocalPlanner::fitPolynomial(const std::vector<Eigen::Vector2d>& waypoints) const
{
    size_t n = waypoints.size();

    if (n < order_ + 1)
    {
        std::cerr << "Not enough points to fit a polynomial of order " << order_ << ".\n";
        return Eigen::VectorXd::Zero(order_ + 1);
    }

    // Initial fit to all points
    Eigen::MatrixXd A(n, order_ + 1);
    Eigen::VectorXd b(n);

    for (auto i = 0; i < n; ++i)
    {
        for (auto j = 0; j < order_ + 1; ++j)
        {
            A(i, j) = pow(waypoints[i].x(), j);
        }
        b(i) = waypoints[i].y();
    }

    Eigen::VectorXd initialCoefficients = A.colPivHouseholderQr().solve(b);

    // Calculate residuals (difference between actual y and fitted y)
    std::vector<double> residuals;
    residuals.reserve(n);

    for (size_t i = 0; i < n; ++i)
    {
        double yFitted = 0.0;
        for (int j = 0; j < order_ + 1; ++j)
        {
            yFitted += initialCoefficients[j] * pow(waypoints[i].x(), j);
        }
        double residual = std::abs(waypoints[i].y() - yFitted);
        residuals.emplace_back(residual);
    }

    // Determine the threshold for outlier removal (e.g., based on a factor of the median residual)
    std::vector<double> sortedResiduals = residuals;
    std::sort(sortedResiduals.begin(), sortedResiduals.end());
    double medianResidual = sortedResiduals[sortedResiduals.size() / 2];
    double threshold      = 2 * medianResidual;  // Can adjust the factor to be more/less strict

    // Remove outliers based on the threshold
    std::vector<Eigen::Vector2d> filteredWaypoints;
    for (size_t i = 0; i < n; ++i)
    {
        if (residuals[i] <= threshold)  // Keep points with residuals below the threshold
        {
            filteredWaypoints.emplace_back(waypoints[i]);
        }
    }

    size_t filtered_n = filteredWaypoints.size();
    if (filtered_n < order_ + 1)
    {
        std::cerr << "Not enough points left after outlier removal to fit a polynomial of order "
                  << order_ << ".\n";
        return Eigen::VectorXd::Zero(order_ + 1);
    }

    // Refit the polynomial to the filtered data
    Eigen::MatrixXd AFiltered(filtered_n, order_ + 1);
    Eigen::VectorXd bFiltered(filtered_n);

    for (auto i = 0; i < filtered_n; ++i)
    {
        for (auto j = 0; j < order_ + 1; ++j)
        {
            AFiltered(i, j) = pow(filteredWaypoints[i].x(), j);
        }
        bFiltered(i) = filteredWaypoints[i].y();
    }

    Eigen::VectorXd finalCoefficients = AFiltered.colPivHouseholderQr().solve(bFiltered);
    return finalCoefficients;
}

std::vector<WayPoint> LocalPlanner::generatePointsWithHeading(const Eigen::VectorXd& coefficient,
                                                              const Pose& currentPose,
                                                              const int& numPoints,
                                                              const double& step)
{
    std::vector<WayPoint> localTrajectory;
    // Save the first point to ego point
    WayPoint wp;
    wp.x     = currentPose.x;
    wp.y     = currentPose.y;
    wp.yaw   = currentPose.yaw;
    wp.v     = 0.0;
    wp.steer = 0.0;

    double x = currentPose.x;
    for (int i = 0; i < numPoints; ++i)
    {
        x += step;
        const double y     = evaluatePolynomial(coefficient, x);
        const double dy_dx = evaluateDerivative(coefficient, x);
        const double yaw   = atan(dy_dx);  // Heading angle in radians

        WayPoint wp;
        wp.x     = x;
        wp.y     = y;
        wp.yaw   = yaw;
        wp.v     = 0.0;
        wp.steer = 0.0;
        localTrajectory.emplace_back(wp);
    }
    return localTrajectory;
}
double LocalPlanner::normalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

double LocalPlanner::evaluatePolynomial(const Eigen::VectorXd& coefficients, const double& x)
{
    double y = 0.0;
    for (int i = 0; i < coefficients.size(); ++i)
    {
        y += coefficients[i] * pow(x, i);
    }
    return y;
}

double LocalPlanner::evaluateDerivative(const Eigen::VectorXd& coefficients, const double& x)
{
    double dy_dx = 0.0;
    for (int i = 1; i < coefficients.size(); ++i)
    {
        dy_dx += i * coefficients[i] * pow(x, i - 1);
    }
    return dy_dx;
}