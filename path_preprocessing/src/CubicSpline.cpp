#include "CubicSpline.h"

CubicSpline::CubicSpline(const std::vector<double>& bx, const std::vector<double>& by)
    : x_(bx),
      y_(by)
{
    assert(x_.size() == y_.size() && x_.size() > 1);
    int n = x_.size() - 1;

    std::vector<double> h(n), alpha(n), l(n + 1), mu(n + 1), z(n + 1);

    // Step 1: Calculate h and alpha
    for (int i = 0; i < n; i++)
    {
        h[i] = x_[i + 1] - x_[i];
        assert(h[i] != 0);
    }

    for (int i = 1; i < n; i++)
    {
        alpha[i] = (3 / h[i]) * (y_[i + 1] - y_[i]) - (3 / h[i - 1]) * (y[i] - y[i - 1]);
    }

    // Step 2: Tridiagonal system setup
    l[0]  = 1.0;
    mu[0] = 0.0;
    z[0]  = 0.0;

    for (int i = 1; i < n; i++)
    {
        l[i]  = 2 * (x_[i + 1] - x_[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i]  = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[n] = 1.0;
    z[n] = 0.0;
    a_.resize(n, 0.0);
    b_.resize(n, 0.0);
    c_.resize(n + 1, 0.0);
    d_.resize(n, 0.0);

    // Step 3: Back substitution
    for (int j = n - 1; j >= 0; j--)
    {
        c_[j] = z[j] - mu[j] * c_[j + 1];
        b_[j] = (y_[j + 1] - y_[j]) / h[j] - h[j] * (c_[j + 1] + 2 * c_[j]) / 3;
        d_[j] = (c_[j + 1] - c_[j]) / (3 * h[j]);
        a_[j] = y_[j];
    }
}

double CubicSpline::evaluate(double xVal)
{
    int n = x_.size() - 1;
    int i = n - 1;
    
    // Find the correct interval for x_val
    for (int j = 0; j < n; j++)
    {
        if (xVal >= x[j] && xVal <= x[j + 1])
        {
            i = j;
            break;
        }
    }

    double dx = xVal - x_[i];
    return a_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
}