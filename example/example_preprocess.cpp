#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "BSpline.h"
#include "CubicSpline.h"
#include "Spline.h"

bool loadBasementData(std::string path, std::vector<Eigen::Vector2d>& points)
{
    if (!std::filesystem::exists(path)) return false;

    std::ifstream data(path);
    std::string line;
    bool isCollect = true;
    while (data)
    {
        std::getline(data, line);
        if (line == "") break;       // check end line \n
        std::stringstream ss(line);  // Create a stringstream from the input string
        std::string token;
        std::vector<double> sample;

        // Use std::getline to split the string by the delimiter
        while (std::getline(ss, token, ' '))
        {
            sample.push_back(std::stod(token));  // Add each token to the result vector
        }
        
        if((int)sample.back() == 1)
        {
            Eigen::Vector2d point(sample[1], sample[2]);
            points.push_back(point);
        }
    }
    return true;
}

// Function to save trajectory to CSV file
void saveToCSV(const std::string& filename, const std::vector<WayPoint>& points)
{
    std::ofstream file(filename);

    if (file.is_open())
    {
        file << "x,y,yaw\n";  // CSV header
        for (const auto& p : points)
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
    if (argc != 4)
    {
        std::cout << "Usage: " << argv[0]
                  << " <method> <input_basement_path> <output_trajectory_path>\n";
        exit(1);
    }

    int method                 = std::stoi(argv[1]);
    std::string basementPath   = argv[2];
    std::string trajectoryPath = argv[3];
    std::vector<Eigen::Vector2d> points;
    bool isLoad = loadBasementData(basementPath, points);
    if (!isLoad)
    {
        std::cout << "Load file error\n";
        return -1;
    }

    std::vector<WayPoint> trajectory;
    double ds  = 0.2;  // Distance between consecutive points
    SplineContext context;
    if (method == 0)    // 0 - B Spline, 1 - Cubic Spline
    {
        BSpline spline;
        context.setStrategy(&spline);
        std::cout << "Data length: " << points.size() << std::endl;
        trajectory = context.interpolate(points, ds);
    }
    else
    {
        CubicSpline spline;
        context.setStrategy(&spline);
        std::cout << "Data length: " << points.size() << std::endl;
        trajectory = context.interpolate(points, ds);
    }
    saveToCSV(trajectoryPath, trajectory);
    return 0;
}
