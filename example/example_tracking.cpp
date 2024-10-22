#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "Controller.h"

bool loadBasementData(std::string path, std::vector<std::tuple<double, double, double>>& points)
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

        std::tuple<double, double, double> point(sample[0], sample[1], sample[2]);  // x, y, yaw
        points.push_back(point);
    }
    return true;
}

// Function to save trajectory to CSV file
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

void getLocalTrajectory(const std::vector<std::tuple<double, double, double>>& points, int iter,
                        std::vector<WayPoint>& localTrajectory)
{
    for (int j = 0; j < N; j++)
    {
        int index = iter + j;
        if (index >= points.size())
        {
            index = points.size() - 1;
        }
        WayPoint wp;
        wp.x     = std::get<0>(points[index]);
        wp.y     = std::get<1>(points[index]);
        wp.yaw   = std::get<2>(points[index]);
        wp.v     = 0.0;
        wp.steer = 0.0;
        localTrajectory.push_back(wp);
    }
}

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "Usage: " << argv[0] << " <input_reference> <output_motion_path>\n";
    }

    std::string referencePath = argv[1];
    std::string motionPath    = argv[2];

    std::vector<std::tuple<double, double, double>> points;
    bool isLoad = loadBasementData(referencePath, points);

    if (!isLoad)
    {
        std::cout << "Load file error\n";
        return -1;
    }

    double dt = 0.1;
    Car car(std::get<0>(points[0]), std::get<1>(points[0]), std::get<2>(points[0]));
    Controller controller;

    for (int i = 0; i < points.size(); i++)
    {
        // Collect reference data
        std::vector<WayPoint> localTrajectory;
        getLocalTrajectory(points, i, localTrajectory);

        // Solve
        ControlSignal signal;
        int success = controller.solve(car.pose, localTrajectory, signal);

        if (success == 0)
        {
            car.updateState(signal, dt);
        }
        std::cout << signal.speed << " " << signal.steering << std::endl;
    }

    saveToCSV(motionPath, car.motionPath);
}