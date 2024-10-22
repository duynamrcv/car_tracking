#pragma once

#include <cmath>
#include <vector>

struct Pose
{
    double x, y, yaw;
};

struct WayPoint
{
    double x;
    double y;
    double yaw;
    double v;
    double steer;
};

struct ControlSignal
{
    double speed;
    double steering;
    double accel;
    int gear;  // -1, 0, 1: backward, neutral, forward
};

class Car
{
public:
    Car(double x_, double y_, double yaw_)
    {
        pose.x   = x_;
        pose.y   = y_;
        pose.yaw = yaw_;
    }

    Car(const Pose& pose_)
    {
        pose = pose_;
    }

    ~Car(){};

    void updateState(const ControlSignal& signal, double dt)
    {
        pose.x   = pose.x + signal.speed * cos(pose.yaw) * dt;
        pose.y   = pose.y + signal.speed * sin(pose.yaw) * dt;
        pose.yaw = pose.yaw + signal.speed * tan(signal.steering) / wheelbase_ * dt;

        motionPath.emplace_back(pose);
    }

    Pose pose;
    std::vector<Pose> motionPath;

private:
    double wheelbase_ = 2.95;
};