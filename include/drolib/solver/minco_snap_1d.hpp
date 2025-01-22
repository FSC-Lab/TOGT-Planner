#ifndef DROLIB_SOLVER_MINCO_SNAP_1D_HPP_
#define DROLIB_SOLVER_MINCO_SNAP_1D_HPP_

#include <Eigen/Eigen>
#include <cmath>
#include <vector>
#include "drolib/polynomial/piecewise_polynomial.hpp"
#include "drolib/solver/banded_system.hpp"

namespace drolib {

// MINCO for s=4 and non-uniform time
class MincoSnap1D {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MincoSnap1D() = default;
  ~MincoSnap1D() { A.destroy(); }

 private:
  int N;
  Eigen::Vector3d headPVAJ;
  Eigen::Vector3d tailPVAJ;
  BandedSystem A;
  Eigen::VectorXd b;
  Eigen::VectorXd T1;
  Eigen::VectorXd T2;
  Eigen::VectorXd T3;
  Eigen::VectorXd T4;
  Eigen::VectorXd T5;
  Eigen::VectorXd T6;
  Eigen::VectorXd T7;

 public:
  void setConditions(const Eigen::Vector3d &headState,
                     const Eigen::Vector3d &tailState,
                     const int &pieceNum) {
    N = pieceNum;
    headPVAJ = headState;
    tailPVAJ = tailState;
    A.create(8 * N, 8, 8);
    b.resize(8 * N);
    T1.resize(N);
    T2.resize(N);
    T3.resize(N);
    T4.resize(N);
    T5.resize(N);
    T6.resize(N);
    T7.resize(N);
    return;
  }


  void setParameters(const Eigen::VectorXd &inPs, const Eigen::VectorXd &ts) {
    T1 = ts;
    T2 = T1.cwiseProduct(T1);
    T3 = T2.cwiseProduct(T1);
    T4 = T2.cwiseProduct(T2);
    T5 = T4.cwiseProduct(T1);
    T6 = T4.cwiseProduct(T2);
    T7 = T4.cwiseProduct(T3);

    A.reset();
    b.setZero();

    A(0, 0) = 1.0;
    A(1, 1) = 1.0;
    A(2, 2) = 2.0;
    A(3, 3) = 6.0;

    b(0) = headPVAJ(0);
    b(1) = headPVAJ(1);
    b(2) = headPVAJ(2);
    b(3) = headPVAJ(3);


    for (int i = 0; i < N - 1; i++) {
      A(8 * i + 4, 8 * i + 4) = 24.0;
      A(8 * i + 4, 8 * i + 5) = 120.0 * T1(i);
      A(8 * i + 4, 8 * i + 6) = 360.0 * T2(i);
      A(8 * i + 4, 8 * i + 7) = 840.0 * T3(i);
      A(8 * i + 4, 8 * i + 12) = -24.0;
      A(8 * i + 5, 8 * i + 5) = 120.0;
      A(8 * i + 5, 8 * i + 6) = 720.0 * T1(i);
      A(8 * i + 5, 8 * i + 7) = 2520.0 * T2(i);
      A(8 * i + 5, 8 * i + 13) = -120.0;
      A(8 * i + 6, 8 * i + 6) = 720.0;
      A(8 * i + 6, 8 * i + 7) = 5040.0 * T1(i);
      A(8 * i + 6, 8 * i + 14) = -720.0;
      A(8 * i + 7, 8 * i) = 1.0;
      A(8 * i + 7, 8 * i + 1) = T1(i);
      A(8 * i + 7, 8 * i + 2) = T2(i);
      A(8 * i + 7, 8 * i + 3) = T3(i);
      A(8 * i + 7, 8 * i + 4) = T4(i);
      A(8 * i + 7, 8 * i + 5) = T5(i);
      A(8 * i + 7, 8 * i + 6) = T6(i);
      A(8 * i + 7, 8 * i + 7) = T7(i);
      A(8 * i + 8, 8 * i) = 1.0;
      A(8 * i + 8, 8 * i + 1) = T1(i);
      A(8 * i + 8, 8 * i + 2) = T2(i);
      A(8 * i + 8, 8 * i + 3) = T3(i);
      A(8 * i + 8, 8 * i + 4) = T4(i);
      A(8 * i + 8, 8 * i + 5) = T5(i);
      A(8 * i + 8, 8 * i + 6) = T6(i);
      A(8 * i + 8, 8 * i + 7) = T7(i);
      A(8 * i + 8, 8 * i + 8) = -1.0;
      A(8 * i + 9, 8 * i + 1) = 1.0;
      A(8 * i + 9, 8 * i + 2) = 2.0 * T1(i);
      A(8 * i + 9, 8 * i + 3) = 3.0 * T2(i);
      A(8 * i + 9, 8 * i + 4) = 4.0 * T3(i);
      A(8 * i + 9, 8 * i + 5) = 5.0 * T4(i);
      A(8 * i + 9, 8 * i + 6) = 6.0 * T5(i);
      A(8 * i + 9, 8 * i + 7) = 7.0 * T6(i);
      A(8 * i + 9, 8 * i + 9) = -1.0;
      A(8 * i + 10, 8 * i + 2) = 2.0;
      A(8 * i + 10, 8 * i + 3) = 6.0 * T1(i);
      A(8 * i + 10, 8 * i + 4) = 12.0 * T2(i);
      A(8 * i + 10, 8 * i + 5) = 20.0 * T3(i);
      A(8 * i + 10, 8 * i + 6) = 30.0 * T4(i);
      A(8 * i + 10, 8 * i + 7) = 42.0 * T5(i);
      A(8 * i + 10, 8 * i + 10) = -2.0;
      A(8 * i + 11, 8 * i + 3) = 6.0;
      A(8 * i + 11, 8 * i + 4) = 24.0 * T1(i);
      A(8 * i + 11, 8 * i + 5) = 60.0 * T2(i);
      A(8 * i + 11, 8 * i + 6) = 120.0 * T3(i);
      A(8 * i + 11, 8 * i + 7) = 210.0 * T4(i);
      A(8 * i + 11, 8 * i + 11) = -6.0;


      b(8 * i + 7) = inPs(i);
    }

    A(8 * N - 4, 8 * N - 8) = 1.0;
    A(8 * N - 4, 8 * N - 7) = T1(N - 1);
    A(8 * N - 4, 8 * N - 6) = T2(N - 1);
    A(8 * N - 4, 8 * N - 5) = T3(N - 1);
    A(8 * N - 4, 8 * N - 4) = T4(N - 1);
    A(8 * N - 4, 8 * N - 3) = T5(N - 1);
    A(8 * N - 4, 8 * N - 2) = T6(N - 1);
    A(8 * N - 4, 8 * N - 1) = T7(N - 1);
    A(8 * N - 3, 8 * N - 7) = 1.0;
    A(8 * N - 3, 8 * N - 6) = 2.0 * T1(N - 1);
    A(8 * N - 3, 8 * N - 5) = 3.0 * T2(N - 1);
    A(8 * N - 3, 8 * N - 4) = 4.0 * T3(N - 1);
    A(8 * N - 3, 8 * N - 3) = 5.0 * T4(N - 1);
    A(8 * N - 3, 8 * N - 2) = 6.0 * T5(N - 1);
    A(8 * N - 3, 8 * N - 1) = 7.0 * T6(N - 1);
    A(8 * N - 2, 8 * N - 6) = 2.0;
    A(8 * N - 2, 8 * N - 5) = 6.0 * T1(N - 1);
    A(8 * N - 2, 8 * N - 4) = 12.0 * T2(N - 1);
    A(8 * N - 2, 8 * N - 3) = 20.0 * T3(N - 1);
    A(8 * N - 2, 8 * N - 2) = 30.0 * T4(N - 1);
    A(8 * N - 2, 8 * N - 1) = 42.0 * T5(N - 1);
    A(8 * N - 1, 8 * N - 5) = 6.0;
    A(8 * N - 1, 8 * N - 4) = 24.0 * T1(N - 1);
    A(8 * N - 1, 8 * N - 3) = 60.0 * T2(N - 1);
    A(8 * N - 1, 8 * N - 2) = 120.0 * T3(N - 1);
    A(8 * N - 1, 8 * N - 1) = 210.0 * T4(N - 1);


    b(8 * N - 4) = tailPVAJ(0);
    b(8 * N - 3) = tailPVAJ(1);
    b(8 * N - 2) = tailPVAJ(2);
    b(8 * N - 1) = tailPVAJ(3);


    A.factorizeLU();
    A.solve(b);

    return;
  }

  void getEnergyWithGrads(double &energy, Eigen::VectorXd &gdC,
                          Eigen::VectorXd &gdT) const {
    // getEnergy(energy);
    // getEnergyPartialGradByCoeffs(gdC);
    // getEnergyPartialGradByTimes(gdT);
  }

  void getEnergy(double &energy) const {
    energy = 0.0;
    for (int i = 0; i < N; i++) {
      energy += 576.0 * pow(b(8 * i + 4), 2) * T1(i) +
                2880.0 * b(8 * i + 4) * (b(8 * i + 5)) * T2(i) +
                4800.0 * pow(b(8 * i + 5), 2) * T3(i) +
                5760.0 * b(8 * i + 4) * (b(8 * i + 6)) * T3(i) +
                21600.0 * b(8 * i + 5) * (b(8 * i + 6)) * T4(i) +
                10080.0 * b(8 * i + 4) * (b(8 * i + 7)) * T4(i) +
                25920.0 * pow(b(8 * i + 6), 2) * T5(i) +
                40320.0 * b(8 * i + 5) * (b(8 * i + 7)) * T5(i) +
                100800.0 * b(8 * i + 6) * (b(8 * i + 7)) * T6(i) +
                100800.0 * pow(b(8 * i + 7), 2) * T7(i);
    }
    return;
  }

  const Eigen::VectorXd &getCoeffs(void) const { return b; }


  void getEnergyPartialGradByCoeffs(Eigen::VectorXd &gdC) const {
    gdC.resize(8 * N);
    for (int i = 0; i < N; i++) {
      gdC(8 * i + 7) = 10080.0 * b(8 * i + 4) * T4(i) +
                           40320.0 * b(8 * i + 5) * T5(i) +
                           100800.0 * b(8 * i + 6) * T6(i) +
                           201600.0 * b(8 * i + 7) * T7(i);
      gdC(8 * i + 6) = 5760.0 * b(8 * i + 4) * T3(i) +
                           21600.0 * b(8 * i + 5) * T4(i) +
                           51840.0 * b(8 * i + 6) * T5(i) +
                           100800.0 * b(8 * i + 7) * T6(i);
      gdC(8 * i + 5) = 2880.0 * b(8 * i + 4) * T2(i) +
                           9600.0 * b(8 * i + 5) * T3(i) +
                           21600.0 * b(8 * i + 6) * T4(i) +
                           40320.0 * b(8 * i + 7) * T5(i);
      gdC(8 * i + 4) = 1152.0 * b(8 * i + 4) * T1(i) +
                           2880.0 * b(8 * i + 5) * T2(i) +
                           5760.0 * b(8 * i + 6) * T3(i) +
                           10080.0 * b(8 * i + 7) * T4(i);
      gdC.segment<4>(8 * i).setZero();
    }
    return;
  }


  void getEnergyPartialGradByTimes(Eigen::VectorXd &gdT) const {
    // gdT.resize(N);
    for (int i = 0; i < N; i++) {
      gdT(i) = 576.0 * pow(b(8 * i + 4), 2) +
               5760.0 * b(8 * i + 4) * (b(8 * i + 5)) * T1(i) +
               14400.0 * pow(b(8 * i + 5), 2) * T2(i) +
               17280.0 * b(8 * i + 4) * (b(8 * i + 6)) * T2(i) +
               86400.0 * b(8 * i + 5) * (b(8 * i + 6)) * T3(i) +
               40320.0 * b(8 * i + 4) * (b(8 * i + 7)) * T3(i) +
               129600.0 * pow(b(8 * i + 6), 2) * T4(i) +
               201600.0 * b(8 * i + 5) * (b(8 * i + 7)) * T4(i) +
               604800.0 * b(8 * i + 6) * (b(8 * i + 7)) * T5(i) +
               705600.0 * pow(b(8 * i + 7), 2) * T6(i);
    }
    return;
  }

void propagateGrad(const Eigen::VectorXd &partialGradByCoeffs,
                     const Eigen::VectorXd &partialGradByTimes,
                     Eigen::VectorXd &gradByPoints,
                     Eigen::VectorXd &gradByTimes) {
    gradByPoints.resize(N - 1);
    gradByTimes.resize(N);
    Eigen::VectorXd adjGrad = partialGradByCoeffs;
    A.solveAdj(adjGrad);

    for (int i = 0; i < N - 1; i++) {
      gradByPoints(i) = adjGrad(8 * i + 7);
    }

    Eigen::Matrix<double, 8, 1> B1;
    Eigen::Matrix<double, 4, 1> B2;
    for (int i = 0; i < N - 1; i++) {
      // negative velocity
      B1(3) =
          -(b(i * 8 + 1) + 2.0 * T1(i) * b(i * 8 + 2) +
            3.0 * T2(i) * b(i * 8 + 3) + 4.0 * T3(i) * b(i * 8 + 4) +
            5.0 * T4(i) * b(i * 8 + 5) + 6.0 * T5(i) * b(i * 8 + 6) +
            7.0 * T6(i) * b(i * 8 + 7));
      B1(4) = B1(3);

      // negative acceleration
      B1(5) =
          -(2.0 * b(i * 8 + 2) + 6.0 * T1(i) * b(i * 8 + 3) +
            12.0 * T2(i) * b(i * 8 + 4) + 20.0 * T3(i) * b(i * 8 + 5) +
            30.0 * T4(i) * b(i * 8 + 6) + 42.0 * T5(i) * b(i * 8 + 7));

      // negative jerk
      B1(6) =
          -(6.0 * b(i * 8 + 3) + 24.0 * T1(i) * b(i * 8 + 4) +
            60.0 * T2(i) * b(i * 8 + 5) + 120.0 * T3(i) * b(i * 8 + 6) +
            210.0 * T4(i) * b(i * 8 + 7));

      // negative snap
      B1(7) = -(24.0 * b(i * 8 + 4) + 120.0 * T1(i) * b(i * 8 + 5) +
                    360.0 * T2(i) * b(i * 8 + 6) +
                    840.0 * T3(i) * b(i * 8 + 7));

      // negative crackle
      B1(0) =
          -(120.0 * b(i * 8 + 5) + 720.0 * T1(i) * b(i * 8 + 6) +
            2520.0 * T2(i) * b(i * 8 + 7));

      // negative d_crackle
      B1(1) =
          -(720.0 * b(i * 8 + 6) + 5040.0 * T1(i) * b(i * 8 + 7));

      // negative dd_crackle
      B1(2) = -5040.0 * b(i * 8 + 7);

      gradByTimes(i) = B1.cwiseProduct(adjGrad.block<8, 1>(8 * i + 4, 0)).sum();
    }

    // negative velocity
    B2(0) = -(b(8 * N - 7) + 2.0 * T1(N - 1) * b(8 * N - 6) +
                  3.0 * T2(N - 1) * b(8 * N - 5) +
                  4.0 * T3(N - 1) * b(8 * N - 4) +
                  5.0 * T4(N - 1) * b(8 * N - 3) +
                  6.0 * T5(N - 1) * b(8 * N - 2) +
                  7.0 * T6(N - 1) * b(8 * N - 1));

    // negative acceleration
    B2(1) = -(2.0 * b(8 * N - 6) + 6.0 * T1(N - 1) * b(8 * N - 5) +
                  12.0 * T2(N - 1) * b(8 * N - 4) +
                  20.0 * T3(N - 1) * b(8 * N - 3) +
                  30.0 * T4(N - 1) * b(8 * N - 2) +
                  42.0 * T5(N - 1) * b(8 * N - 1));

    // negative jerk
    B2(2) = -(6.0 * b(8 * N - 5) + 24.0 * T1(N - 1) * b(8 * N - 4) +
                  60.0 * T2(N - 1) * b(8 * N - 3) +
                  120.0 * T3(N - 1) * b(8 * N - 2) +
                  210.0 * T4(N - 1) * b(8 * N - 1));

    // negative snap
    B2(3) =
        -(24.0 * b(8 * N - 4) + 120.0 * T1(N - 1) * b(8 * N - 3) +
          360.0 * T2(N - 1) * b(8 * N - 2) +
          840.0 * T3(N - 1) * b(8 * N - 1));

    gradByTimes(N - 1) =
        B2.cwiseProduct(adjGrad.segment<4>(8 * N - 4)).sum();
    gradByTimes += partialGradByTimes;
  }
};

}  // namespace drolib


#endif  /* DROLIB_SOLVER_MINCO_SNAP_1D_HPP_ */

