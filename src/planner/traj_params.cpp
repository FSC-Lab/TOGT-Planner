#include "drolib/planner/traj_params.hpp"

namespace drolib {

bool TrajParams::load(const Yaml& yaml) {
#define GET_PARAM(name)                   \
  if (!yaml[#name].getIfDefined(name)) {  \
    std::cout << #name << " is not found" \
              << "\n";                    \
    return false;                         \
  }

  GET_PARAM(piecesPerSegment)
  GET_PARAM(speedGuess)
  GET_PARAM(maxVelNorm)
  GET_PARAM(maxOmgXY)
  GET_PARAM(maxOmgZ)
  GET_PARAM(maxTiltedAngle)
  GET_PARAM(maxThr)
  GET_PARAM(minThr)

  GET_PARAM(boundX)
  GET_PARAM(boundY)
  GET_PARAM(boundZ)

  GET_PARAM(weightTime)
  GET_PARAM(weightEnergy)
  GET_PARAM(weightPos)
  GET_PARAM(weightVel)
  GET_PARAM(weightOmg)
  GET_PARAM(weightRot)
  GET_PARAM(weightThr)
  GET_PARAM(smoothingEps)
  GET_PARAM(numConstPena)
  GET_PARAM(dynamicConstCheck)
  GET_PARAM(minNumCheck)
  GET_PARAM(maxNumCheck)
  GET_PARAM(checkTimeSec)

#undef GET_PARAM

  maxVelSqr = maxVelNorm * maxVelNorm;
  maxOmgXYSqr = maxOmgXY * maxOmgXY;
  maxOmgZSqr = maxOmgZ * maxOmgZ;
  thrMean = 0.5 * (maxThr + minThr);
  thrRadi = 0.5 * (maxThr - minThr);
  thrRadiSqr = thrRadi * thrRadi;

  collectivtThrMean = 4.0 * thrMean;
  collectivtThrRadi = 4.0 * thrRadi;
  collectivtThrRadiSqr = collectivtThrRadi * collectivtThrRadi;
  return true;
}

bool TrajParams::valid() const {
  bool check = true;
  check &= piecesPerSegment > 0;
  check &= speedGuess > 0.0;
  check &= maxVelNorm > 0.0;
  check &= maxOmgXY > 0.0;
  check &= maxOmgZ > 0.0;
  check &= maxTiltedAngle > 0.0;
  check &= maxThr > 0.0;
  check &= minThr >= 0.0;
  check &= boundX.allFinite();
  check &= boundY.allFinite();
  check &= boundZ.allFinite();
  check &= weightTime >= 0;
  check &= weightEnergy >= 0;
  check &= weightPos >= 0;
  check &= weightOmg >= 0;
  check &= weightRot >= 0;
  check &= weightThr >= 0;
  check &= smoothingEps >= 0;
  check &= numConstPena >= 0;
  check &= minNumCheck >= 0;
  check &= maxNumCheck >= 0;
  check &= checkTimeSec > 0;

  return check;
}

std::ostream& operator<<(std::ostream& os, const TrajParams& params) {
  os.precision(3);
  // os << std::scientific;
  os << "TrajParams:\n"
     << "piecesPerSegment =    [" << params.piecesPerSegment << "]\n"
     << "speedGuess =          [" << params.speedGuess << "]\n"
     << "maxVelNorm =          [" << params.maxVelNorm << "]\n"
     << "maxOmgXY =            [" << params.maxOmgXY << "]\n"
     << "maxOmgZ =             [" << params.maxOmgZ << "]\n"
     << "maxTiltedAngle =      [" << params.maxTiltedAngle << "]\n"
     << "maxThr =              [" << params.maxThr << "]\n"
     << "minThr =              [" << params.minThr << "]\n"
     << "weightTime =          [" << params.weightTime << "]\n"
     << "weightEnergy =        [" << params.weightEnergy << "]\n"
     << "weightPos =           [" << params.weightPos << "]\n"
     << "weightVel =           [" << params.weightVel << "]\n"
     << "weightOmg =           [" << params.weightOmg << "]\n"
     << "weightRot =           [" << params.weightRot << "]\n"
     << "weightThr =           [" << params.weightThr << "]\n"
     << "maxOmgXY =            [" << params.maxOmgXY << "]\n"
     << "maxOmgZ =             [" << params.maxOmgZ << "]\n"
     << "smoothingEps =        [" << params.smoothingEps << "]\n"
     << "numConstPena =        [" << params.numConstPena << "]\n"
     << "boundX =              [" << params.boundX.transpose() << "]\n"
     << "boundY =              [" << params.boundY.transpose() << "]\n"
     << "boundZ =              [" << params.boundZ.transpose() << "]"
     << std::endl;
  os.precision();
  // os.unsetf(std::ios::scientific);
  return os;
}

}  // namespace drolib


