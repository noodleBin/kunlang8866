
#include "quintic_spline_2d.h"

using Eigen::VectorXd;

namespace century {
namespace planning {


// -------------------- QuinticSpline1D --------------------
QuinticSpline1D::QuinticSpline1D(const std::vector<double>& x,
                                 const std::vector<double>& y,
                                 const std::vector<double>& dy,
                                 const std::vector<double>& ddy)
    : x_(x), y_(y), dy_(dy), ddy_(ddy) {
  if (x.size() < 2 || y.size() < 2 || x.size() != y.size() ||
      y.size() != dy.size() || dy.size() != ddy.size()) {
    throw std::invalid_argument("Invalid input size for QuinticSpline1D.");
  }
  ComputeCoefficients();
}

void QuinticSpline1D::ComputeCoefficients() {
  coeffs_.clear();
  int n = x_.size() - 1;
  for (int i = 0; i < n; ++i) {
    double x0 = x_[i];
    double x1 = x_[i + 1];
    double h = x1 - x0;

    double p0 = y_[i];
    double p1 = y_[i + 1];
    double v0 = dy_[i];
    double v1 = dy_[i + 1];
    double a0 = ddy_[i];
    double a1 = ddy_[i + 1];

    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 1> B;
    A << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 1, h, h * h,
        h * h * h, pow(h, 4), pow(h, 5), 0, 1, 2 * h, 3 * h * h, 4 * pow(h, 3),
        5 * pow(h, 4), 0, 0, 2, 6 * h, 12 * h * h, 20 * pow(h, 3);
    B << p0, v0, a0, p1, v1, a1;
    Eigen::Matrix<double, 6, 1> coeff = A.colPivHouseholderQr().solve(B);
    coeffs_.push_back(
        {coeff(0), coeff(1), coeff(2), coeff(3), coeff(4), coeff(5)});
  }
}

int QuinticSpline1D::FindSegment(double t) const {
  if (t <= x_.front()) {
    return 0;
  }
  if (t >= x_.back()) {
    return x_.size() - 2;
  }
  auto it = std::upper_bound(x_.begin(), x_.end(), t);
  return static_cast<int>(std::distance(x_.begin(), it)) - 1;
}

double QuinticSpline1D::Calc(double t) const {
  int i = FindSegment(t);
  double s = t - x_[i];
  const auto& c = coeffs_[i];
  return c.a0 + c.a1 * s + c.a2 * pow(s, 2) + c.a3 * pow(s, 3) +
         c.a4 * pow(s, 4) + c.a5 * pow(s, 5);
}

double QuinticSpline1D::CalcFirstDeriv(double t) const {
  int i = FindSegment(t);
  double s = t - x_[i];
  const auto& c = coeffs_[i];
  return c.a1 + 2 * c.a2 * s + 3 * c.a3 * pow(s, 2) + 4 * c.a4 * pow(s, 3) +
         5 * c.a5 * pow(s, 4);
}

double QuinticSpline1D::CalcSecondDeriv(double t) const {
  int i = FindSegment(t);
  double s = t - x_[i];
  const auto& c = coeffs_[i];
  return 2 * c.a2 + 6 * c.a3 * s + 12 * c.a4 * pow(s, 2) +
         20 * c.a5 * pow(s, 3);
}

double QuinticSpline1D::CalcThirdDeriv(double t) const {
  int i = FindSegment(t);
  double s = t - x_[i];
  const auto& c = coeffs_[i];
  return 6 * c.a3 + 24 * c.a4 * s + 60 * c.a5 * pow(s, 2);
}

// -------------------- QuinticSpline2D --------------------
QuinticSpline2D::QuinticSpline2D(const std::vector<double>& s,
                                 const std::vector<double>& x,
                                 const std::vector<double>& y) {
  std::vector<double> dx(x.size(), 0.0);
  std::vector<double> ddx(x.size(), 0.0);
  std::vector<double> dy_vec(y.size(), 0.0);
  std::vector<double> ddy_vec(y.size(), 0.0);

  sx_ = QuinticSpline1D(s, x, dx, ddx);
  sy_ = QuinticSpline1D(s, y, dy_vec, ddy_vec);
}

void QuinticSpline2D::CalcPosition(double s_query, double* x, double* y) const {
  *x = sx_.Calc(s_query);
  *y = sy_.Calc(s_query);
}

void QuinticSpline2D::CalcDerivatives(double s_query, double* dx, double* dy,
                                      double* ddx, double* ddy, double* dddx,
                                      double* dddy) const {
  *dx = sx_.CalcFirstDeriv(s_query);
  *dy = sy_.CalcFirstDeriv(s_query);
  *ddx = sx_.CalcSecondDeriv(s_query);
  *ddy = sy_.CalcSecondDeriv(s_query);
  *dddx = sx_.CalcThirdDeriv(s_query);
  *dddy = sy_.CalcThirdDeriv(s_query);
}

}  // namespace planning
}  // namespace century