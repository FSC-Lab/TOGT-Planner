#pragma once

#include <Eigen/Eigen>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <vector>
#include "drolib/solver/root_finder.hpp"

namespace drolib {

template <int D>
class Polynomial {
 public:
  using CoefficientMat = Eigen::Matrix<double, 3, D + 1>;
  using VelCoefficientMat = Eigen::Matrix<double, 3, D>;
  using AccCoefficientMat = Eigen::Matrix<double, 3, D - 1>;

 private:
  double duration;
  CoefficientMat coeffMat;

 public:
  Polynomial() = default;

  Polynomial(double dur, const CoefficientMat &cMat)
      : duration(dur), coeffMat(cMat) {}

  inline int getDim() const { return 3; }

  inline int getDegree() const { return D; }

  inline double getDuration() const { return duration; }

  inline const CoefficientMat &getCoeffMat() const { return coeffMat; }

  VelCoefficientMat getVelCoeffMat() const {
    VelCoefficientMat velCoeffMat;
    int n = 1;
    for (int i = D - 1; i >= 0; i--) {
      velCoeffMat.col(i) = n * coeffMat.col(i);
      n++;
    }
    return velCoeffMat;
  }

  Eigen::Vector3d getPos(const double &t) const {
    Eigen::Vector3d pos(0.0, 0.0, 0.0);
    double tn = 1.0;
    for (int i = D; i >= 0; i--) {
      pos += tn * coeffMat.col(i);
      tn *= t;
    }
    return pos;
  }

  Eigen::Vector3d getVel(const double &t) const {
    Eigen::Vector3d vel(0.0, 0.0, 0.0);
    double tn = 1.0;
    int n = 1;
    for (int i = D - 1; i >= 0; i--) {
      vel += n * tn * coeffMat.col(i);
      tn *= t;
      n++;
    }
    return vel;
  }

  Eigen::Vector3d getAcc(const double &t) const {
    Eigen::Vector3d acc(0.0, 0.0, 0.0);
    double tn = 1.0;
    int m = 1;
    int n = 2;
    for (int i = D - 2; i >= 0; i--) {
      acc += m * n * tn * coeffMat.col(i);
      tn *= t;
      m++;
      n++;
    }
    return acc;
  }

  Eigen::Vector3d getJer(const double &t) const {
    Eigen::Vector3d jer(0.0, 0.0, 0.0);
    double tn = 1.0;
    int l = 1;
    int m = 2;
    int n = 3;
    for (int i = D - 3; i >= 0; i--) {
      jer += l * m * n * tn * coeffMat.col(i);
      tn *= t;
      l++;
      m++;
      n++;
    }
    return jer;
  }

  Eigen::Vector3d getSna(const double &t) const {
    Eigen::Vector3d sna(0.0, 0.0, 0.0);
    double tn = 1.0;
    int l = 1;
    int m = 2;
    int n = 3;
    int o = 4;
    for (int i = D - 4; i >= 0; i--) {
      sna += l * m * n * o * tn * coeffMat.col(i);
      tn *= t;
      l++;
      m++;
      n++;
      o++;
    }
    return sna;
  }

  CoefficientMat normalizePosCoeffMat() const {
    CoefficientMat nPosCoeffsMat;
    double t = 1.0;
    for (int i = D; i >= 0; i--) {
      nPosCoeffsMat.col(i) = coeffMat.col(i) * t;
      t *= duration;
    }
    return nPosCoeffsMat;
  }

  VelCoefficientMat normalizeVelCoeffMat() const {
    VelCoefficientMat nVelCoeffMat;
    int n = 1;
    double t = duration;
    for (int i = D - 1; i >= 0; i--) {
      nVelCoeffMat.col(i) = n * coeffMat.col(i) * t;
      t *= duration;
      n++;
    }
    return nVelCoeffMat;
  }

  AccCoefficientMat normalizeAccCoeffMat() const {
    AccCoefficientMat nAccCoeffMat;
    int n = 2;
    int m = 1;
    double t = duration * duration;
    for (int i = D - 2; i >= 0; i--) {
      nAccCoeffMat.col(i) = n * m * coeffMat.col(i) * t;
      n++;
      m++;
      t *= duration;
    }
    return nAccCoeffMat;
  }

  double getMaxVel() const {
    VelCoefficientMat nVelCoeffMat = normalizeVelCoeffMat();
    Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                            RootFinder::polySqr(nVelCoeffMat.row(1)) +
                            RootFinder::polySqr(nVelCoeffMat.row(2));
    int N = coeff.size();
    int n = N - 1;
    for (int i = 0; i < N; i++) {
      coeff(i) *= n;
      n--;
    }
    if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON) {
      return getVel(0.0).norm();
    } else {
      double l = -0.0625;
      double r = 1.0625;
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON) {
        l = 0.5 * l;
      }
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON) {
        r = 0.5 * (r + 1.0);
      }
      std::set<double> candidates = RootFinder::solvePolynomial(
          coeff.head(N - 1), l, r, FLT_EPSILON / duration);
      candidates.insert(0.0);
      candidates.insert(1.0);
      double maxVelSqr = -INFINITY;
      double tempNormSqr;
      for (std::set<double>::const_iterator it = candidates.begin();
           it != candidates.end(); it++) {
        if (0.0 <= *it && 1.0 >= *it) {
          tempNormSqr = getVel((*it) * duration).squaredNorm();
          maxVelSqr = maxVelSqr < tempNormSqr ? tempNormSqr : maxVelSqr;
        }
      }
      return sqrt(maxVelSqr);
    }
  }

  double getMaxAcc() const {
    AccCoefficientMat nAccCoeffMat = normalizeAccCoeffMat();
    Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                            RootFinder::polySqr(nAccCoeffMat.row(1)) +
                            RootFinder::polySqr(nAccCoeffMat.row(2));
    int N = coeff.size();
    int n = N - 1;
    for (int i = 0; i < N; i++) {
      coeff(i) *= n;
      n--;
    }
    if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON) {
      return getAcc(0.0).norm();
    } else {
      double l = -0.0625;
      double r = 1.0625;
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON) {
        l = 0.5 * l;
      }
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON) {
        r = 0.5 * (r + 1.0);
      }
      std::set<double> candidates = RootFinder::solvePolynomial(
          coeff.head(N - 1), l, r, FLT_EPSILON / duration);
      candidates.insert(0.0);
      candidates.insert(1.0);
      double maxAccSqr = -INFINITY;
      double tempNormSqr;
      for (std::set<double>::const_iterator it = candidates.begin();
           it != candidates.end(); it++) {
        if (0.0 <= *it && 1.0 >= *it) {
          tempNormSqr = getAcc((*it) * duration).squaredNorm();
          maxAccSqr = maxAccSqr < tempNormSqr ? tempNormSqr : maxAccSqr;
        }
      }
      return sqrt(maxAccSqr);
    }
  }

  bool checkMaxVel(const double &maxVel) const {
    double sqrMaxVel = maxVel * maxVel;
    if (getVel(0.0).squaredNorm() >= sqrMaxVel ||
        getVel(duration).squaredNorm() >= sqrMaxVel) {
      return false;
    } else {
      VelCoefficientMat nVelCoeffMat = normalizeVelCoeffMat();
      Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                              RootFinder::polySqr(nVelCoeffMat.row(1)) +
                              RootFinder::polySqr(nVelCoeffMat.row(2));
      double t2 = duration * duration;
      coeff.tail<1>()(0) -= sqrMaxVel * t2;
      return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
    }
  }

  bool checkMaxAcc(const double &maxAcc) const {
    double sqrMaxAcc = maxAcc * maxAcc;
    if (getAcc(0.0).squaredNorm() >= sqrMaxAcc ||
        getAcc(duration).squaredNorm() >= sqrMaxAcc) {
      return false;
    } else {
      AccCoefficientMat nAccCoeffMat = normalizeAccCoeffMat();
      Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                              RootFinder::polySqr(nAccCoeffMat.row(1)) +
                              RootFinder::polySqr(nAccCoeffMat.row(2));
      double t2 = duration * duration;
      double t4 = t2 * t2;
      coeff.tail<1>()(0) -= sqrMaxAcc * t4;
      return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
    }
  }

  double getClosestPoint(const Eigen::Vector3d &pt, double &tt,
                         Eigen::Vector3d &projectedPoint) {
    // std::cout << "piece getClosestPoint\n";                     
    // 2*(p-p0)^T * \dot{p} = 0
    auto l_coeff = getCoeffMat();
    // std::cout << "l_coeff: \n" << l_coeff << "\n";

    // std::cout << "l_coeff.col(D): " << l_coeff.col(D) << "\n";
    l_coeff.col(D) = l_coeff.col(D) - pt;

    auto r_coeff = getVelCoeffMat();
    // std::cout << "r_coeff: \n" << r_coeff << "\n";


    Eigen::VectorXd eq = Eigen::VectorXd::Zero(2 * D);
    for (int j = 0; j < l_coeff.rows(); ++j) {
      eq = eq + RootFinder::polyConv(l_coeff.row(j), r_coeff.row(j));
    }

    double l = -0.0625;
    double r = duration + 0.0625;

    while (fabs(RootFinder::polyVal(eq, l)) < DBL_EPSILON) {
      l = 0.5 * l;
    }

    while (fabs(RootFinder::polyVal(eq, r)) < DBL_EPSILON) {
      r = 0.5 * (duration + r);
    }

    std::set<double> roots = RootFinder::solvePolynomial(eq, l, r, 1e-6);
    // std::cout << "# roots: " << roots.size() << std::endl;
    double minDist = -1;
    for (const auto &root : roots) {
      // std::cout << "root: " << root << std::endl;
      if (root < 0 || root > duration) {
        continue;
      }
      if (getVel(root).norm() < 1e-6) {  // velocity == 0, ignore it
        continue;
      }
      // std::cout << "find min!" << std::endl;
      Eigen::Vector3d p = getPos(root);
      // std::cout << "p: " << p.transpose() << std::endl;
      double distance = (p - pt).norm();
      if (distance < minDist || minDist < 0) {
        minDist = distance;
        tt = root;
        projectedPoint = p;
      }
    }
    return minDist;
  }

  bool intersectPlane(const Eigen::Vector3d p, const Eigen::Vector3d v,
                      double &tt, Eigen::Vector3d &pt) const {
    // (pt - p)^T * v = 0
    auto coeff = getCoeffMat();
    coeff.col(D) = coeff.col(D) - p;  // reset the origin to p
    Eigen::VectorXd eq =
        coeff.transpose() * v;  // v is the normal vector of the plan
    double l = -0.0625;
    double r = duration + 0.0625;
    while (fabs(RootFinder::polyVal(eq, l)) < DBL_EPSILON) {
      l = 0.5 * l;
    }
    while (fabs(RootFinder::polyVal(eq, r)) < DBL_EPSILON) {
      r = 0.5 * (duration + r);
    }
    std::set<double> roots = RootFinder::solvePolynomial(eq, l, r, 1e-6);
    for (const auto &root : roots) {
      tt = root;
      pt = getPos(root);
      return true;
    }
    return false;
  }
};

}  // namespace drolib