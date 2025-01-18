#pragma once

#include <Eigen/Eigen>
#include "drolib/base/parameter_base.hpp"

namespace drolib {

class TrajParams : public ParameterBase {
 public:
  TrajParams() {
    maxVelSqr = maxVelNorm * maxVelNorm;
    maxOmgXYSqr = maxOmgXY * maxOmgXY;
    maxOmgZSqr = maxOmgZ * maxOmgZ;
    thrMean = 0.5 * (maxThr + minThr);
    thrRadi = 0.5 * (maxThr - minThr);
    thrRadiSqr = thrRadi * thrRadi;
  
    collectivtThrMean = 4.0 * thrMean;
    collectivtThrRadi = 4.0 * thrRadi;
    collectivtThrRadiSqr = collectivtThrRadi * collectivtThrRadi;    
  }

  using ParameterBase::load;
  bool load(const Yaml& yaml) override;
  bool valid() const override;

  friend std::ostream& operator<<(std::ostream& os, const TrajParams& params);

 public:
  int piecesPerSegment{2};
  double speedGuess{1};

  double maxVelNorm{100};
  double maxOmgXY{15};
  double maxOmgZ{3};
  double maxTiltedAngle{6.28};
  double maxThr{6.88};
  double minThr{0.1};

  double weightTime{1.0};
  double weightEnergy{0.1};
  double weightPos{1.0};
  double weightVel{1.0};
  double weightOmg{1.0};
  double weightRot{1.0};
  double weightThr{1.0};
  double weightPerception{0.0};

  double smoothingEps{1.0e-2};
  int numConstPena{16};
  bool dynamicConstCheck{false};
  int minNumCheck{8};
  int maxNumCheck{32};
  double checkTimeSec{0.05};

  double maxVelSqr{};
  double maxOmgXYSqr{};
  double maxOmgZSqr{};
  double thrMean{};
  double thrRadi{};
  double thrRadiSqr{};

  double collectivtThrMean{};
  double collectivtThrRadi{};
  double collectivtThrRadiSqr{};

  Eigen::Vector2d boundX{-10000,10000};
  Eigen::Vector2d boundY{-10000,10000};
  Eigen::Vector2d boundZ{-10000,10000};

  Eigen::Quaterniond q_bc{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d t_b_cb{Eigen::Vector3d::Zero()};
};

}  // namespace drolib