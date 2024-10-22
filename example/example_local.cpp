#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "Controller.h"
#include "LocalPlanner.h"

bool loadBasementData(std::string path, std::vector<WayPoint>& points)
{
    if (!std::filesystem::exists(path)) return false;

    std::ifstream data(path);
    std::string line;
    bool isCollect = true;
    std::getline(data, line);  // Do not consider the first line
    while (data)
    {
        std::getline(data, line);
        if (line == "") break;       // check end line \n
        std::stringstream ss(line);  // Create a stringstream from the input string
        std::string token;
        std::vector<double> sample;

        // Use std::getline to split the string by the delimiter
        while (std::getline(ss, token, ','))
        {
            sample.push_back(std::stod(token));  // Add each token to the result vector
        }

        WayPoint wp;
        wp.x     = sample[0];
        wp.y     = sample[1];
        wp.yaw   = sample[2];
        wp.v     = 0.0;
        wp.steer = 0.0;
        points.emplace_back(wp);
    }
    return true;
}

// Function to save trajectory to CSV file
void saveToCSV(const std::string& filename, const std::vector<std::vector<WayPoint>>& data)
{
    std::ofstream file(filename);

    if (file.is_open())
    {
        // file << "x,y,yaw\n";  // CSV header
        for (const std::vector<WayPoint>& path : data)
        {
            for (const WayPoint& p : path)
            {
                file << p.x << " " << p.y << " " << p.yaw << " ";
            }
            file << "\n";
        }
        file.close();
        std::cout << "Trajectory saved to " << filename << std::endl;
    }
    else
    {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
}

void saveToCSV(const std::string& filename,
               const std::vector<Pose>& motionPath)
{
    std::ofstream file(filename);

    if (file.is_open())
    {
        file << "x,y,yaw\n";  // CSV header
        for (const auto& p : motionPath)
        {
            file << p.x << "," << p.y << "," << p.yaw << "\n";
        }
        file.close();
        std::cout << "Trajectory saved to " << filename << std::endl;
    }
    else
    {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
}

int main(int argc, char** argv)
{
    // Step 1: Read global path from file
    if (argc != 3)
    {
        std::cout << "Usage: " << argv[0]
                  << " <input_reference> <output_local_path> <output_motion_path>\n";
    }

    std::string referencePath = argv[1];
    std::string localPath     = argv[2];
    std::string motionPath    = argv[3];

    std::vector<WayPoint> globalPath;
    bool isLoad = loadBasementData(referencePath, globalPath);

    if (!isLoad)
    {
        std::cout << "Load file error\n";
        return -1;
    }

    // Step 2: Initialize vehicle pose at a starting point
    Pose vehiclePose;
    vehiclePose.x   = globalPath[0].x;
    vehiclePose.y   = globalPath[0].y+1.0;
    vehiclePose.yaw = globalPath[0].yaw;

    double dt = 0.1;
    Car car(vehiclePose);
    Controller controller;
    LocalPlanner lp(globalPath, car.pose, 3);

    // Step 3: Animate vehicle movement along the path using genLocalPathInter
    std::vector<std::vector<WayPoint>> data;
    double totalTime   = 50.0;
    double currentTime = 0.0;
    // for (size_t i = 0; i < globalPath.size() - 30; ++i)
    while (currentTime < totalTime)
    {
        currentTime += 0.1;
        // Use genLocalPathInter to get points with heading
        std::vector<WayPoint> pointsWithHeading = lp.genLocalPathInter(car.pose, 2 * N, N, 0.1);

        // Convert the generated local path back to global coordinates
        std::vector<WayPoint> globalPathConverted = lp.convertLocalToGlobal(pointsWithHeading);

        // Solve
        ControlSignal signal;
        int success = controller.solve(car.pose, globalPathConverted, signal);

        if (success == 0)
        {
            car.updateState(signal, dt);
        }
        std::cout << signal.speed << " " << signal.steering << std::endl;

        data.emplace_back(globalPathConverted);
    }

    // Save to CSV
    saveToCSV(motionPath, car.motionPath);
    saveToCSV(localPath, data);

    return 0;
}