#pragma once

#include <Eigen/Eigen>
#include "drolib/base/parameter_base.hpp"

namespace drolib {

class TrajParams : public ParameterBase {
 public:
  TrajParams() = default;

  using ParameterBase::load;
  bool load(const Yaml& yaml) override;
  bool valid() const override;

  friend std::ostream& operator<<(std::ostream& os, const TrajParams& params);

 public:
  int piecesPerSegment{};
  double speedGuess{};

  double maxVelNorm{};
  double maxOmgXY{};
  double maxOmgZ{};
  double maxTiltedAngle{};
  double maxThr{};
  double minThr{};

  double weightTime{};
  double weightEnergy{};
  double weightPos{};
  double weightVel{};
  double weightOmg{};
  double weightRot{};
  double weightThr{};

  double smoothingEps{};
  int numConstPena{};
  bool dynamicConstCheck{false};
  int minNumCheck{};
  int maxNumCheck{};
  double checkTimeSec{};

  double maxVelSqr{};
  double maxOmgXYSqr{};
  double maxOmgZSqr{};
  double thrMean{};
  double thrRadi{};
  double thrRadiSqr{};

  double collectivtThrMean{};
  double collectivtThrRadi{};
  double collectivtThrRadiSqr{};

  Eigen::Vector2d boundX;
  Eigen::Vector2d boundY;
  Eigen::Vector2d boundZ;
};

}  // namespace drolib