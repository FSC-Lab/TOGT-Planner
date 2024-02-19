#include "drolib/system/quadrotor_params.hpp"

namespace drolib {
  
bool QuadParams::load(const Yaml& yaml) {
  if (yaml.isNull()) return false;

#define GET_PARAM(name)                   \
  if (!yaml[#name].getIfDefined(name)) {  \
    std::cout << #name << " is not found" \
              << "\n";                    \
    return false;                         \
  }

  GET_PARAM(mass)
  GET_PARAM(inertia)
#undef GET_PARAM
  T_bm.resize(4, 4);
  T_mb.resize(4, 4);

  if (yaml["armLength"].isDefined() &&
      yaml["beta"].isDefined() &&
      yaml["torCoeff"].isDefined()) {
    const double l = yaml["armLength"].as<double>();
    const double beta = yaml["beta"].as<double>();
    const double ct = yaml["torCoeff"].as<double>();

    const double lsb = l * sin(beta * M_PI / 180.0);
    const double lcb = l * cos(beta * M_PI / 180.0);
    T_bm = (Eigen::Matrix4d() << 
      1.0, 1.0, 1.0, 1.0,
      -lsb, lsb, -lsb, lsb,
      -lcb, lcb, lcb, -lcb,
      -ct, -ct, ct, ct).finished();

    T_mb = T_bm.inverse();

  } else if (yaml["T_bm"].isDefined()) {
    yaml["T_bm"] >> T_bm;
    // T_mb.resize(4,4);
    // T_mb.col(0) << 0.25, 0.25, 0.25, 0.25;
    // T_mb.col(1) << 1.66666667, -1.66666667, -1.66666667, 1.66666667;
    // T_mb.col(2) << -1.66666667, -1.66666667, 1.66666667, 1.66666667;
    // T_mb.col(3) << 25, -25, 25, -25;
    T_mb = T_bm.inverse();
  } else if (yaml["tbm_fr"].isDefined() && yaml["tbm_bl"].isDefined() &&
             yaml["tbm_br"].isDefined() && yaml["tbm_fl"].isDefined() &&
             yaml["torCoeff"].isDefined()) {
    Eigen::Matrix<double, 3, 4> tbm;
    tbm(0, 0) = yaml["tbm_fr"][0].as<double>();
    tbm(1, 0) = yaml["tbm_fr"][1].as<double>();
    tbm(2, 0) = yaml["tbm_fr"][2].as<double>();

    tbm(0, 1) = yaml["tbm_bl"][0].as<double>();
    tbm(1, 1) = yaml["tbm_bl"][1].as<double>();
    tbm(2, 1) = yaml["tbm_bl"][2].as<double>();

    tbm(0, 2) = yaml["tbm_br"][0].as<double>();
    tbm(1, 2) = yaml["tbm_br"][1].as<double>();
    tbm(2, 2) = yaml["tbm_br"][2].as<double>();

    tbm(0, 3) = yaml["tbm_fl"][0].as<double>();
    tbm(1, 3) = yaml["tbm_fl"][1].as<double>();
    tbm(2, 3) = yaml["tbm_fl"][2].as<double>();

    const double ct = yaml["torCoeff"].as<double>();

    T_bm = (Eigen::Matrix4d() << 
                Eigen::Vector4d::Ones().transpose(), 
                tbm.row(1),
                -tbm.row(0),
                ct * Eigen::Vector4d(-1, -1, 1, 1).transpose()).finished();

    T_mb = T_bm.inverse();
  } else if (yaml["tbm_r0"].isDefined() && yaml["tbm_r1"].isDefined() &&
             yaml["tbm_r2"].isDefined() && yaml["tbm_r3"].isDefined()) {
    T_bm(0, 0) = yaml["tbm_r0"][0].as<double>();
    T_bm(0, 1) = yaml["tbm_r0"][1].as<double>();
    T_bm(0, 2) = yaml["tbm_r0"][2].as<double>();
    T_bm(0, 3) = yaml["tbm_r0"][3].as<double>();

    T_bm(1, 0) = yaml["tbm_r1"][0].as<double>();
    T_bm(1, 1) = yaml["tbm_r1"][1].as<double>();
    T_bm(1, 2) = yaml["tbm_r1"][2].as<double>();
    T_bm(1, 3) = yaml["tbm_r1"][3].as<double>();

    T_bm(2, 0) = yaml["tbm_r2"][0].as<double>();
    T_bm(2, 1) = yaml["tbm_r2"][1].as<double>();
    T_bm(2, 2) = yaml["tbm_r2"][2].as<double>();
    T_bm(2, 3) = yaml["tbm_r2"][3].as<double>();

    T_bm(3, 0) = yaml["tbm_r3"][0].as<double>();
    T_bm(3, 1) = yaml["tbm_r3"][1].as<double>();
    T_bm(3, 2) = yaml["tbm_r3"][2].as<double>();
    T_bm(3, 3) = yaml["tbm_r3"][3].as<double>();
    
    T_mb = T_bm.inverse();
  } else {
    return false;
  }

  return true;
}

bool QuadParams::valid() const {
  bool check = true;
  check &= mass > 0.0;
  check &= mass < 100.0;
  check &= inertia.allFinite();
  check &= T_mb.allFinite();
  check &= T_bm.allFinite();

  return check;
}

std::ostream& operator<<(std::ostream& os, const QuadParams& params) {
  os.precision(4);
  // os << std::scientific;
  os << "QuadParams:\n"
     << "mass =       [" << params.mass << "]\n"
     << "inertia =    [" << params.inertia.transpose() << "]\n"
     << "T_mb =  \n   [" << params.T_mb << "]\n"
     << "T_bm =  \n   [" << params.T_bm << "]"
     << std::endl;
  os.precision();
  // os.unsetf(std::ios::scientific);
  return os;
}


}  // namespace drolib