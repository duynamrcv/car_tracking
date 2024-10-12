#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include <cmath>

// Function to calculate the B-spline basis function
double B(int i, int k, double t, const std::vector<double>& knots) {
    if (k == 0) {
        return (t >= knots[i] && t < knots[i + 1]) ? 1.0 : 0.0;
    } else {
        double left = 0.0;
        double right = 0.0;

        double denominator_left = knots[i + k] - knots[i];
        if (denominator_left != 0) {
            left = (t - knots[i]) / denominator_left * B(i, k - 1, t, knots);
        }

        double denominator_right = knots[i + k + 1] - knots[i + 1];
        if (denominator_right != 0) {
            right = (knots[i + k + 1] - t) / denominator_right * B(i + 1, k - 1, t, knots);
        }

        return left + right;
    }
}

// Function to generate knot vector
std::vector<double> generateKnotVector(int numPoints, int degree) {
    int n = numPoints + degree + 1;
    std::vector<double> knots(n);

    for (int i = 0; i <= degree; i++) {
        knots[i] = 0.0;
    }
    for (int i = degree + 1; i < n - degree - 1; i++) {
        knots[i] = static_cast<double>(i - degree) / (n - 2 * degree);
    }
    for (int i = n - degree - 1; i < n; i++) {
        knots[i] = 1.0;
    }

    return knots;
}

// Function to compute Euclidean distance between two points
double computeDistance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
    return std::sqrt(std::pow(p2.x() - p1.x(), 2) + std::pow(p2.y() - p1.y(), 2));
}

double computeBaselineDistance(const std::vector<Eigen::Vector2d>& points)
{
    double distance = 0;
    for (int i = 0; i < points.size()-1; ++i)
    {
        distance += computeDistance(points[i], points[i+1]);
    }
    return distance;
}

// Function to perform 2D cubic B-spline interpolation
std::vector<Eigen::Vector2d> cubicBSpline(const std::vector<Eigen::Vector2d>& points, double ds) {
    int degree = 3;  // Cubic spline

    // Generate knot vector
    std::vector<double> knots = generateKnotVector(points.size(), degree);

    // Initialize the output points
    std::vector<Eigen::Vector2d> splinePoints;

    double t = 0.0;          // Parameter to traverse the spline
    double totalArcLength = 0.0;
    Eigen::Vector2d prevPoint = points[points.size()-1];

    // Compute baseline distance
    double baselineLength = computeBaselineDistance(points);
    double step = ds/(20*baselineLength);

    // Loop to generate the spline based on the distance `ds`
    while (t < 1.0) {
        Eigen::Vector2d p(0.0, 0.0);

        // Compute spline point for parameter t
        for (int i = 0; i < points.size(); ++i) {
            double basisValue = B(i, degree, t, knots);
            p += basisValue * points[i];
        }

        // Calculate the distance from the previous point
        double dist = computeDistance(prevPoint, p);
        // totalArcLength += dist;

        // If the distance exceeds the specified `ds`, store the point
        if (dist >= ds) {
            splinePoints.push_back(p);
            prevPoint = p;  // Update the previous point to the current point
        }

        // Increment t based on the current arc length and ds (increase more for large ds)
        // t += ds / totalArcLength;
        t += step;
    }

    return splinePoints;
}

// Function to save trajectory to CSV file
void saveToCSV(const std::string& filename, const std::vector<Eigen::Vector2d>& points) {
    std::ofstream file(filename);

    if (file.is_open()) {
        file << "x,y\n";  // CSV header
        for (const auto& p : points) {
            file << p.x() << "," << p.y() << "\n";
        }
        file.close();
        std::cout << "Trajectory saved to " << filename << std::endl;
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
}

int main() {
    // Define input points (control points)
    std::vector<Eigen::Vector2d> points = {
        Eigen::Vector2d(-1.0, 0.0),
        Eigen::Vector2d(3.0, -3.0),
        Eigen::Vector2d(4.0, 1.0),
        Eigen::Vector2d(2.0, 1.0),
        Eigen::Vector2d(1.0, 3.0)
    };

    double ds = 0.1;  // Distance between consecutive points (not used explicitly in this basic version)

    // Perform cubic B-spline interpolation
    std::vector<Eigen::Vector2d> splinePoints = cubicBSpline(points, ds);

    // Save the trajectory to a CSV file
    saveToCSV("trajectory.csv", splinePoints);

    return 0;
}
