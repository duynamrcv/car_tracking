// #include <filesystem>
// #include <fstream>
// #include <iostream>
// #include <sstream>
// #include <vector>

// bool loadBasementData(std::string path, std::vector<float>& bx, std::vector<float>& by)
// {
//     if (!std::filesystem::exists(path)) return false;

//     std::ifstream data(path);
//     std::string line;
//     while (data)
//     {
//         std::getline(data, line);
//         if (line == "") break;       // check end line \n
//         std::stringstream ss(line);  // Create a stringstream from the input string
//         std::string token;
//         std::vector<float> sample(9);

//         // Use std::getline to split the string by the delimiter
//         while (std::getline(ss, token, ' '))
//         {
//             sample.emplace_back(std::stof(token));  // Add each token to the result vector
//         }
//         bx.push_back(sample[1]);
//         by.push_back(sample[2]);
//     }
//     return true;
// }

// int main(int argc, char** argv)
// {
//     if (argc != 2)
//     {
//         std::cout << "Usage: " << argv[0] << " <sample_path>\n";
//     }

//     std::string basementPath = argv[1];
//     std::vector<float> bx, by;
//     bool isLoad = loadBasementData(basementPath, bx, by);
//     if (!isLoad) std::cout << "Load file error\n";

//     std::cout << "Data length: " << bx.size() << std::endl;
//     return 0;
// }

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>  // For std::upper_bound
#include <fstream>
#include <Eigen/Dense>  // For matrix operations (like numpy in Python)

using namespace std;
using namespace Eigen;

class Spline {
public:
    vector<double> a, b, c, d;  // Coefficients
    vector<double> x, y;        // Data points
    int nx;

    Spline(const vector<double>& x_, const vector<double>& y_) : x(x_), y(y_) {
        nx = x.size();
        vector<double> h(nx - 1);

        // Differences between consecutive x points
        for (int i = 0; i < nx - 1; i++) {
            h[i] = x[i + 1] - x[i];
        }

        // Calculate coefficients
        a = y;

        // Solve for c
        MatrixXd A = calc_A(h);
        VectorXd B = calc_B(h);

        VectorXd c_vec = A.colPivHouseholderQr().solve(B);
        c = vector<double>(c_vec.data(), c_vec.data() + c_vec.size());

        // Calculate b and d
        for (int i = 0; i < nx - 1; i++) {
            double tb = (a[i + 1] - a[i]) / h[i] - h[i] * (2.0 * c[i] + c[i + 1]) / 3.0;
            b.push_back(tb);
            d.push_back((c[i + 1] - c[i]) / (3.0 * h[i]));
        }
    }

    double calc(double t) {
        // Calculate spline position for t
        if (t < x[0] || t > x[nx - 1]) return NAN;
        int i = search_index(t);
        double dx = t - x[i];
        return a[i] + b[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx;
    }

    double calcd(double t) {
        // Calculate first derivative for t
        if (t < x[0] || t > x[nx - 1]) return NAN;
        int i = search_index(t);
        double dx = t - x[i];
        return b[i] + 2.0 * c[i] * dx + 3.0 * d[i] * dx * dx;
    }

    double calcdd(double t) {
        // Calculate second derivative for t
        if (t < x[0] || t > x[nx - 1]) return NAN;
        int i = search_index(t);
        double dx = t - x[i];
        return 2.0 * c[i] + 6.0 * d[i] * dx;
    }

private:
    int search_index(double t) {
        // Find the index of the segment containing t
        auto it = upper_bound(x.begin(), x.end(), t);
        return max(0, int(it - x.begin()) - 1);
    }

    MatrixXd calc_A(const vector<double>& h) {
        // Calculate matrix A for spline coefficients
        MatrixXd A = MatrixXd::Zero(nx, nx);
        A(0, 0) = 1.0;
        for (int i = 1; i < nx - 1; i++) {
            A(i, i - 1) = h[i - 1];
            A(i, i) = 2.0 * (h[i - 1] + h[i]);
            A(i, i + 1) = h[i];
        }
        A(nx - 1, nx - 1) = 1.0;
        return A;
    }

    VectorXd calc_B(const vector<double>& h) {
        // Calculate vector B for spline coefficients
        VectorXd B = VectorXd::Zero(nx);
        for (int i = 1; i < nx - 1; i++) {
            B(i) = 3.0 * ((a[i + 1] - a[i]) / h[i] - (a[i] - a[i - 1]) / h[i - 1]);
        }
        return B;
    }
};

class Spline2D {
public:
    Spline sx, sy;
    vector<double> s;  // Arc lengths

    Spline2D(const vector<double>& x, const vector<double>& y) : s(calc_s(x, y)), sx(calc_s(x, y), x), sy(calc_s(x, y), y) {}

    pair<double, double> calc_position(double s_val) {
        return {sx.calc(s_val), sy.calc(s_val)};
    }

    double calc_curvature(double s_val) {
        double dx = sx.calcd(s_val);
        double ddx = sx.calcdd(s_val);
        double dy = sy.calcd(s_val);
        double ddy = sy.calcdd(s_val);
        return (ddy * dx - ddx * dy) / pow((dx * dx + dy * dy), 1.5);
    }

    double calc_yaw(double s_val) {
        double dx = sx.calcd(s_val);
        double dy = sy.calcd(s_val);
        return atan2(dy, dx);
    }

private:
    vector<double> calc_s(const vector<double>& x, const vector<double>& y) {
        // Calculate the cumulative arc length
        vector<double> ds(x.size() - 1);
        vector<double> s_val = {0};
        for (size_t i = 1; i < x.size(); i++) {
            double dx = x[i] - x[i - 1];
            double dy = y[i] - y[i - 1];
            ds[i - 1] = hypot(dx, dy);
            s_val.push_back(s_val.back() + ds[i - 1]);
        }
        return s_val;
    }
};

vector<pair<double, double>> calc_spline_course(const vector<double>& x, const vector<double>& y, double ds = 0.1) {
    Spline2D sp(x, y);
    vector<pair<double, double>> trajectory;
    double max_s = sp.s.back();

    for (double s_val = 0; s_val < max_s; s_val += ds) {
        trajectory.push_back(sp.calc_position(s_val));
    }
    return trajectory;
}

int main() {
    vector<double> x = {0.0, -3.5, 2.0, 10.0, 3.0, 5.0, -2.0};
    vector<double> y = {0.0, 2.0, -1.5, 5.0, 5.0, 10.0, 3.0};
    double ds = 0.1;

    // Calculate the spline trajectory
    vector<pair<double, double>> trajectory = calc_spline_course(x, y, ds);

    // Save to a file
    ofstream outFile("trajectory.txt");
    if (outFile.is_open()) {
        outFile << "x,y\n";
        for (const auto& point : trajectory) {
            outFile << point.first << "," << point.second << "\n";
        }
        outFile.close();
        cout << "Trajectory saved to 'trajectory.txt'" << endl;
    } else {
        cerr << "Unable to open file" << endl;
    }

    return 0;
}
