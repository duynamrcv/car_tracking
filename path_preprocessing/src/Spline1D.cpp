#include "Spline1D.h"

#include <algorithm>  // For std::upper_bound

Spline1D::Spline1D(const std::vector<double>& x, const std::vector<double>& y) : x_(x), y_(y)
{
    nx_ = x_.size();
    std::vector<double> h(nx_ - 1);

    // Differences between consecutive x points
    for (int i = 0; i < nx_ - 1; i++)
    {
        h[i] = x_[i + 1] - x_[i];
    }

    // Calculate coefficients
    a_ = y;

    // Solve for c
    Eigen::MatrixXd A = computeA(h);
    Eigen::VectorXd B = computeB(h);

    Eigen::VectorXd cVec = A.colPivHouseholderQr().solve(B);
    c_                   = std::vector<double>(cVec.data(), cVec.data() + cVec.size());

    // Calculate b and d
    for (int i = 0; i < nx_ - 1; i++)
    {
        double tb = (a_[i + 1] - a_[i]) / h[i] - h[i] * (2.0 * c_[i] + c_[i + 1]) / 3.0;
        b_.emplace_back(tb);
        d_.emplace_back((c_[i + 1] - c_[i]) / (3.0 * h[i]));
    }
}

double Spline1D::compute(double t)
{
    // Calculate spline position for t
    if (t < x_[0] || t > x_[nx_ - 1]) return NAN;
    int i     = searchIndex(t);
    double dx = t - x_[i];
    return a_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
}

double Spline1D::computeD(double t)
{
    // Calculate first derivative for t
    if (t < x_[0] || t > x_[nx_ - 1]) return NAN;
    int i     = searchIndex(t);
    double dx = t - x_[i];
    return b_[i] + 2.0 * c_[i] * dx + 3.0 * d_[i] * dx * dx;
}

double Spline1D::computeDD(double t)
{
    // Calculate second derivative for t
    if (t < x_[0] || t > x_[nx_ - 1]) return NAN;
    int i     = searchIndex(t);
    double dx = t - x_[i];
    return 2.0 * c_[i] + 6.0 * d_[i] * dx;
}

int Spline1D::searchIndex(double t)
{
    // Find the index of the segment containing t
    auto it = std::upper_bound(x_.begin(), x_.end(), t);
    return std::max(0, int(it - x_.begin()) - 1);
}

Eigen::MatrixXd Spline1D::computeA(const std::vector<double> h)
{
    // Calculate matrix A for spline coefficients
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nx_, nx_);
    A(0, 0)           = 1.0;
    for (int i = 1; i < nx_ - 1; i++)
    {
        A(i, i - 1) = h[i - 1];
        A(i, i)     = 2.0 * (h[i - 1] + h[i]);
        A(i, i + 1) = h[i];
    }
    A(nx_ - 1, nx_ - 1) = 1.0;
    return A;
}

Eigen::MatrixXd Spline1D::computeB(const std::vector<double> h)
{
    // Calculate vector B for spline coefficients
    Eigen::VectorXd B = Eigen::VectorXd::Zero(nx_);
    for (int i = 1; i < nx_ - 1; i++)
    {
        B(i) = 3.0 * ((a_[i + 1] - a_[i]) / h[i] - (a_[i] - a_[i - 1]) / h[i - 1]);
    }
    return B;
}