#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

namespace century {
namespace planning {

class QuinticSpline1D {
 public:
  QuinticSpline1D() = default;
  QuinticSpline1D(const std::vector<double>& x, const std::vector<double>& y,
                  const std::vector<double>& dy,
                  const std::vector<double>& ddy);

  double Calc(double t) const;
  double CalcFirstDeriv(double t) const;
  double CalcSecondDeriv(double t) const;
  double CalcThirdDeriv(double t) const;

 private:
  std::vector<double> x_;
  struct Coeff {
    double a0, a1, a2, a3, a4, a5;
  };
  std::vector<Coeff> coeffs_;

  int FindSegment(double t) const;
  void ComputeCoefficients();
  std::vector<double> y_, dy_, ddy_;
};

class QuinticSpline2D {
 public:
  QuinticSpline2D(const std::vector<double>& s, const std::vector<double>& x,
                  const std::vector<double>& y);

  void CalcPosition(double s_query, double* x, double* y) const;
  void CalcDerivatives(double s_query, double* dx, double* dy, double* ddx,
                       double* ddy, double* dddx, double* dddy) const;

 private:
  QuinticSpline1D sx_, sy_;
};

}  // namespace planning
}  // namespace century