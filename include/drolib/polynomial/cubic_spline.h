#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdio>
#include <vector>

namespace drolib {

namespace spline {

// band matrix solver
class BandMatrix {
private:
  std::vector<std::vector<double>> m_upper; // upper band
  std::vector<std::vector<double>> m_lower; // lower band
public:
  BandMatrix(){};                         // constructor
  BandMatrix(int dim, int n_u, int n_l);  // constructor
  ~BandMatrix(){};                        // destructor
  void resize(int dim, int n_u, int n_l); // init with dim,n_u,n_l
  int dim() const;                        // matrix dimension
  int num_upper() const { return m_upper.size() - 1; }
  int num_lower() const { return m_lower.size() - 1; }
  // access operator
  double &operator()(int i, int j);      // write
  double operator()(int i, int j) const; // read
  // we can store an additional diogonal (in m_lower)
  double &saved_diag(int i);
  double saved_diag(int i) const;
  void lu_decompose();
  std::vector<double> r_solve(const std::vector<double> &b) const;
  std::vector<double> l_solve(const std::vector<double> &b) const;
  std::vector<double> lu_solve(const std::vector<double> &b,
                               bool is_lu_decomposed = false);
};

// CubicSpline interpolation
class CubicSpline {
public:
  enum bd_type { first_deriv = 1, second_deriv = 2 };

private:
  std::vector<double> m_x, m_y; // x,y coordinates of points
  // interpolation parameters
  // f(x) = a*(x-x_i)^3 + b*(x-x_i)^2 + c*(x-x_i) + y_i
  std::vector<double> m_a, m_b, m_c; // CubicSpline coefficients
  double m_b0, m_c0;                 // for left extrapol
  bd_type m_left, m_right;
  double m_left_value, m_right_value;
  bool m_force_linear_extrapolation;

public:
  // set default boundary condition to be zero curvature at both ends
  CubicSpline()
      : m_left(second_deriv), m_right(second_deriv), m_left_value(0.0),
        m_right_value(0.0), m_force_linear_extrapolation(false) {}

  // optional, but if called it has to come be before set_points()
  void set_boundary(bd_type left, double left_value, bd_type right,
                    double right_value,
                    bool force_linear_extrapolation = false);

  void set_points(const std::vector<double> &x, const std::vector<double> &y,
                  bool cubic_spline = true);

  double operator()(double x) const;

  double deriv(int order, double x) const;

  void coeffs(double x, double &x_s, std::vector<double> &poly_coeff) const;

  std::array<double, 5> coeffs(double x) const;

  bool sample_points(double x, double &y_s, double &y_e);
};

} // namespace spline

} // namespace drolib
