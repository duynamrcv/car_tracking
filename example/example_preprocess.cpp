#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "BSpline.h"
#include "CubicSpline.h"
#include "Spline.h"

bool loadBasementData(std::string path, std::vector<std::vector<Eigen::Vector2d>>& listPoints)
{
    if (!std::filesystem::exists(path)) return false;

    std::ifstream data(path);
    std::string line;
    bool isCollect = true;
    int prevGear   = 1;
    std::vector<Eigen::Vector2d> points;
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

        int curGear = (int)sample.back();
        if (curGear == prevGear || curGear == 0)
        {
            Eigen::Vector2d point(sample[1], sample[2]);
            points.push_back(point);
        }
        else
        {
            Eigen::Vector2d point(sample[1], sample[2]);
            std::cout << "Point: " << sample[1] << ", " << sample[2] << std::endl;
            points.push_back(point);
            listPoints.push_back(points);  // add points to the list

            points.clear();
            points.push_back(point);
            prevGear = curGear;
        }
    }
    listPoints.push_back(points);  // add points to the list
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

// Function to calculate the perpendicular distance from a point to a line
double perpendicularDistance(const Eigen::Vector2d& p, const Eigen::Vector2d& lineStart,
                             const Eigen::Vector2d& lineEnd)
{
    double num =
        std::abs((lineEnd.y() - lineStart.y()) * p.x() - (lineEnd.x() - lineStart.x()) * p.y() +
                 lineEnd.x() * lineStart.y() - lineEnd.y() * lineStart.x());
    double den = std::sqrt(std::pow(lineEnd.y() - lineStart.y(), 2) +
                           std::pow(lineEnd.x() - lineStart.x(), 2));
    return num / den;
}

// Recursive Ramer-Douglas-Peucker algorithm
void rdp(const std::vector<Eigen::Vector2d>& points, std::vector<Eigen::Vector2d>& result,
         const double& epsilon = 0.1)
{
    if (points.size() < 2) return;

    double maxDist = 0.0;
    size_t index   = 0;

    // Find the point with the maximum distance from the baseline
    for (size_t i = 1; i < points.size() - 1; ++i)
    {
        double dist = perpendicularDistance(points[i], points.front(), points.back());
        if (dist > maxDist)
        {
            index   = i;
            maxDist = dist;
        }
    }

    // If max distance is greater than epsilon, recursively simplify
    if (maxDist > epsilon)
    {
        std::vector<Eigen::Vector2d> left(points.begin(), points.begin() + index + 1);
        std::vector<Eigen::Vector2d> right(points.begin() + index, points.end());

        std::vector<Eigen::Vector2d> resultLeft, resultRight;
        rdp(left, resultLeft, epsilon);
        rdp(right, resultRight, epsilon);

        // Concatenate results (avoid duplicating the middle point)
        result.assign(resultLeft.begin(), resultLeft.end() - 1);
        result.insert(result.end(), resultRight.begin(), resultRight.end());
    }
    else
    {
        // If max distance is less than epsilon, simplify to a straight line
        result.push_back(points.front());
        result.push_back(points.back());
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
    std::vector<std::vector<Eigen::Vector2d>> listPoints;
    bool isLoad = loadBasementData(basementPath, listPoints);
    if (!isLoad)
    {
        std::cout << "Load file error\n";
        return -1;
    }

    double epsilon = 0.05;
    for (size_t i = 0; i < listPoints.size(); i++)
    {
        std::vector<Eigen::Vector2d> points = listPoints[i];
        rdp(points, points, epsilon);

        std::vector<WayPoint> trajectory;
        double ds = 0.1;  // Distance between consecutive points
        SplineContext context;
        if (method == 0)  // 0 - B Spline, 1 - Cubic Spline
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
        saveToCSV(trajectoryPath + std::to_string(i) + ".csv", trajectory);
    }

    return 0;
}
