#pragma once

// #include <deque>
#include "drolib/polynomial/polynomial.hpp"

namespace drolib {

using PVAJ = Eigen::Matrix<double, 3, 4>;
using PVAJS = Eigen::Matrix<double, 3, 5>;

template <int D>
class PiecewisePolynomial {
 private:
  using Pieces = std::vector<Polynomial<D>>;
  Pieces pieces;

 public:
  PiecewisePolynomial() = default;

  PiecewisePolynomial(
      const std::vector<double>& durations,
      const std::vector<typename Polynomial<D>::CoefficientMat>& cMats) {
    const int N = std::min(durations.size(), cMats.size());
    pieces.reserve(N);
    for (int i = 0; i < N; i++) {
      pieces.emplace_back(durations[i], cMats[i]);
    }
  }

  explicit PiecewisePolynomial(const Pieces& polynomials) : pieces(polynomials) {}

  inline int getPieceNum() const { return pieces.size(); }

  inline bool valid() const { return !pieces.empty(); }  

  inline const Polynomial<D> &operator[](int i) const {
      return pieces[i];
  }

  inline Polynomial<D> &operator[](int i) {
      return pieces[i];
  }

  inline void clear(void) { pieces.clear(); }

  int locatePieceIdx(double &t) const {
    const int N = getPieceNum();
    int idx;
    double dur;
    for (idx = 0; idx < N && t > (dur = pieces[idx].getDuration()); idx++) {
      t -= dur;
    }
    if (idx == N) {
      idx--;
      t += pieces[idx].getDuration();
    }
    return idx;
  }

  Eigen::VectorXd getDurations() const {
    const int N = getPieceNum();
    Eigen::VectorXd durations(N);
    int i{0};
    for (const auto& piece : pieces) {
      durations(i) = piece.getDuration();
      ++i;
    }
    return durations;
  }

  Eigen::Matrix3Xd getPoints() const {
    const int N = getPieceNum();
    Eigen::Matrix3Xd positions(3, N + 1);
    int i{0};
    for (const auto& piece : pieces) {
      positions.col(i) = piece.getCoeffMat().col(D);
      ++i;
    }
    positions.col(N) = pieces.back().getPos(pieces.back().getDuration());
    return positions;
  }

  Eigen::Matrix3Xd getWaypoints() const {
    const int N = getPieceNum();
    // if (endpoint = false) {
    //   Eigen::Matrix3Xd positions(3, N - 1);
    //   if (N <= 1) {
    //     return positions;
    //   }
    //   for (int i{1}, j{0}; j < N; ++i, ++j) {
    //     positions.col(j) = pieces[i].getCoeffMat().col(D);
    //   }
    //   return positions;
    // } else {
    Eigen::Matrix3Xd positions(3, N);
    if (N <= 1) {
      positions.col(N - 1) = pieces.back().getPos(pieces.back().getDuration());
      return positions;
    }
    for (int i{1}, j{0}; j < N - 1; ++i, ++j) {
      positions.col(j) = pieces[i].getCoeffMat().col(D);
    }
    positions.col(N - 1) = pieces.back().getPos(pieces.back().getDuration());
    
    return positions;
    // }


  }

  double getTotalDuration() const {
    // const int N = getPieceNum();
    double totalDuration = 0.0;
    for (const auto& piece : pieces) {
      totalDuration += piece.getDuration();
    }
    return totalDuration;
  }

  // typename Pieces::iterator locatePiece(double& t) {
  //   const int N = getPieceNum();
  //   double duration;
  //   typename Pieces::iterator it = pieces.begin();
  //   typename Pieces::const_iterator itEnd = pieces.end();
  //   // TODO: define operator < to simplify the process
  //   for (; it != itEnd && t > (duration = it->getDuration()); ++it) {
  //     t -= duration;
  //   }
  //   if (it == itEnd) {
  //     it--;
  //     t += it->getDuration();
  //   }
  //   return it;
  // }

  // TODO: use const double t
  Eigen::Vector3d getPos(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getPos(t);
  }

  PVAJ getPVAJ(double t) const {
    return (PVAJ() << getPos(t), getVel(t), getAcc(t), getJer(t)).finished();
  }

  PVAJS getPVAJS(double t) const {
    return (PVAJS() << getPos(t), getVel(t), getAcc(t), getJer(t), getSna(t)).finished();
  }

  Eigen::Vector3d getVel(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getVel(t);
  }

  Eigen::Vector3d getAcc(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getAcc(t);
  }

  Eigen::Vector3d getJer(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getJer(t);
  }

  Eigen::Vector3d getSna(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces[pieceIdx].getSna(t);
  }

  double getMaxVel() const {
    // int N = getPieceNum();
    double maxVel = -INFINITY;
    double tempNorm;
    for (const auto& piece : pieces) {
      tempNorm = piece.getMaxVel();
      maxVel = maxVel < tempNorm ? tempNorm : maxVel;
    }
    return maxVel;
  }

  double getMaxAcc() const {
    // int N = getPieceNum();
    double maxAcc = -INFINITY;
    double tempNorm;
    for (const auto& piece : pieces) {
      tempNorm = piece.getMaxAcc();
      maxAcc = maxAcc < tempNorm ? tempNorm : maxAcc;
    }

    return maxAcc;
  }

  bool intersectPlane(const Eigen::Vector3d p, const Eigen::Vector3d v,
                      double& tt, Eigen::Vector3d& pt) {
    for (const auto& piece : pieces) {
      if (piece.intersectPlane(p, v, tt, pt)) {
        return true;
      }
    }
    return false;
  }

  inline typename Pieces::const_iterator begin() const { return pieces.begin(); }

  inline typename Pieces::const_iterator end() const { return pieces.end(); }

  inline typename Pieces::iterator begin() { return pieces.begin(); }

  inline typename Pieces::iterator end() { return pieces.end(); }
  
  inline void reserve(const int &n) { pieces.reserve(n); }

  inline void emplace_back(const double& duration,
                           const typename Polynomial<D>::CoefficientMat& cMat) {
    pieces.emplace_back(duration, cMat);
  }

  inline void emplace_front(const double& duration,
                           const typename Polynomial<D>::CoefficientMat& cMat) {
    pieces.emplace_front(duration, cMat);
  }

  inline void append_back(const PiecewisePolynomial<D>& polys) {
    pieces.insert(pieces.end(), polys.begin(), polys.end());
  }

  inline void append_front(const PiecewisePolynomial<D>& polys) {
    pieces.insert(pieces.begin(), polys.begin(), polys.end());
  }

  inline void erase_front(const int n) {
    pieces.erase(pieces.begin(), pieces.begin() + n);
  }

  inline void erase_back(const int n) {
    auto it = pieces.end();
    for (int i{n}; i > 0; --i, --it);
    pieces.erase(it, pieces.end());
  }

  inline void push_back(const Polynomial<D>& poly) { pieces.push_back(poly); }

  Pieces pop_back(const int n = 1) {
    assert(this->getPieceNum() >= n);
    Pieces polynomials;
    for (int i{0}; i < n; ++i) {
      polynomials.push_back(pieces.back());
      pieces.pop_back();
    }
    return polynomials;
  }

  PiecewisePolynomial<D> getPieces(const int idx, int num = -1) const {
    if (num == -1) {
      num = this->getPieceNum() - idx;
    }
    assert(idx + num <= this->getPieceNum());
    Pieces polynomials;
    for (int i{0}; i < num; ++i) {
      polynomials.push_back(pieces[idx + i]);
    }
    return PiecewisePolynomial<D>(polynomials);
  }

  // GaaiLam
  double getClosestPoint(const Eigen::Vector3d &pt,
                         int &ii, double &tt,
                         Eigen::Vector3d &projectedPoint) {
    // std::cout << "pt: " << pt.transpose() << "\n";                    
    double minDist = -1;
    for (int i = 0; i < getPieceNum(); ++i) {
      auto piece = pieces[i];
      double t = 0;
      double dist = piece.getClosestPoint(pt, t, projectedPoint);
      if (dist < 0) {
        continue;
      }
      if (minDist < 0 || dist < minDist) {
        minDist = dist;
        ii = i;
        tt = t;
      }
    }
    return minDist;
  }

  bool intersectPlane(const Eigen::Vector3d p,
                      const Eigen::Vector3d v,
                      int &ii, double &tt, Eigen::Vector3d &pt) {
    for (int i = 0; i < getPieceNum(); ++i) {
      const auto &piece = pieces[i];
      if (piece.intersectPlane(p, v, tt, pt)) {
        ii = i;
        return true;
      }
    }
    return false;
  }


  // Eigen::Matrix3Xd waypoints() {
  //   Eigen::Matrix3Xd points;
  //   points.resize(3, getPieceNum() + 1);
  //   int i{0};
  //   auto it = pieces.begin();
  //   for (; it != pieces.end(); ++it, ++i) {
  //     points.col(i) = it->getPos(0.0);
  //     // points.push_back(it->getPos(0.0));
  //   }
  //   it--;
  //   points.col(i) = it->getPos(it->getDuration());
  //   // points.push_back(it->getPos(it->getDuration()));
  //   return points;
  // }

};

}  // namespace drolib