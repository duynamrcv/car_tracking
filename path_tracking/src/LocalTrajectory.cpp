#include "LocalTrajectory.h"

LocalTrajectory::LocalTrajectory(const std::vector<WayPoint>& globalPath, const Pose& pose,
                                 const int order)
{
    globalPath_  = globalPath;
    vehiclePose_ = pose;
    order_       = order;
}
LocalTrajectory::~LocalTrajectory() {}

void LocalTrajectory::updateVehiclePose(const Pose& pose)
{
    vehiclePose_ = pose;
}

void LocalTrajectory::findClosestWaypointAhead()
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

std::vector<WayPoint> LocalTrajectory::getLocalPathAhead(const int& numPoints)
{
    std::vector<WayPoint> localPath;  // Ego waypoint
    WayPoint ego;
    ego.x     = 0.0;
    ego.y     = 0.0;
    ego.yaw   = 0.0;
    ego.v     = 0.0;
    ego.steer = 0.0;
    localPath.emplace_back(ego);

    findClosestWaypointAhead();

    for (size_t i = vehicleIndex_; i < vehicleIndex_ + numPoints && i < globalPath_.size(); ++i)
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
    while (localPath.size() < static_cast<size_t>(numPoints) + 1)
    {
        // +1 because of ego-point
        localPath.emplace_back(localPath.back());
    }

    return localPath;
}

std::vector<WayPoint> LocalTrajectory::getGlobalPathAhead(const int& numPoints)
{
    std::vector<WayPoint> globalPathAhead;
    WayPoint ego;
    ego.x     = vehiclePose_.x;
    ego.y     = vehiclePose_.y;
    ego.yaw   = vehiclePose_.yaw;
    ego.v     = 0.0;
    ego.steer = 0.0;
    globalPathAhead.emplace_back(ego);

    findClosestWaypointAhead();

    for (size_t i = vehicleIndex_; i < vehicleIndex_ + numPoints && i < globalPath_.size(); ++i)
    {
        globalPathAhead.emplace_back(globalPath_[i]);
    }

    // If not enough points, add the last point to match the required number of points
    while (globalPathAhead.size() < static_cast<size_t>(numPoints) + 1)
    {
        // +1 because of ego-point
        globalPathAhead.emplace_back(globalPathAhead.back());
    }

    return globalPathAhead;
}

std::vector<WayPoint> LocalTrajectory::convertLocalToGlobal(
    const std::vector<WayPoint>& localTrajectory) const
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

Eigen::VectorXd LocalTrajectory::fitPolynomial(const std::vector<Eigen::Vector2d>& waypoints) const
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

std::vector<WayPoint> LocalTrajectory::generatePointsWithHeading(const Eigen::VectorXd& coefficient,
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
    for (int i = 1; i < numPoints; ++i)
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
double LocalTrajectory::normalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

double LocalTrajectory::evaluatePolynomial(const Eigen::VectorXd& coefficients, const double& x)
{
    double y = 0.0;
    for (int i = 0; i < coefficients.size(); ++i)
    {
        y += coefficients[i] * pow(x, i);
    }
    return y;
}

double LocalTrajectory::evaluateDerivative(const Eigen::VectorXd& coefficients, const double& x)
{
    double dy_dx = 0.0;
    for (int i = 1; i < coefficients.size(); ++i)
    {
        dy_dx += i * coefficients[i] * pow(x, i - 1);
    }
    return dy_dx;
}