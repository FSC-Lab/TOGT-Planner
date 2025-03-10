#pragma once

#include "drolib/base/corridor_base.hpp"
#include "drolib/shape/hexagon.h"
#include "drolib/shape/pentagon.h"
#include "drolib/shape/rectangle.h"
#include "drolib/shape/triangle.h"
#include <unordered_map>

namespace drolib {

class PrismaCorridor : public CorridorBase {
public:
  std::string shapeName;
  double length{0.0};
  int midpoints{0};

  // TODO(chao): hack for square gate
  double outer_side{0};
  double inner_side{0};
  double a_, b_, d_, m_, r_;

  PrismaCorridor();

  PrismaCorridor(const std::string shape, const Yaml &yaml,
                 const std::string &order = "");

  Eigen::Vector3d getCenter() override {
    const Polygon &in =
        dynamic_cast<const Polygon &>(*corridor.front().shape.get());
    const Polygon &out =
        dynamic_cast<const Polygon &>(*corridor.back().shape.get());
    Eigen::Vector3d position = (in.position + out.position) * 0.5;
    return position;
  }

  Eigen::Quaterniond getOrientation() override {
    const Polygon &in =
        dynamic_cast<const Polygon &>(*corridor.front().shape.get());
    return in.R_wb;
  }

  Eigen::Vector3d getRotationEulerRPY() override {
    const Polygon &in =
        dynamic_cast<const Polygon &>(*corridor.front().shape.get());
    Eigen::Vector3d rpy = rad2deg(quaternionToEulerAnglesRPY(in.R_wb));
    return rpy;
  }

  double calcDistance(const Eigen::Vector3d &pos,
                      Eigen::Vector3d &gradDistanceToPos) override {
    // TODO(chao): only support square gate now
    const Polygon &in =
        dynamic_cast<const Polygon &>(*corridor.front().shape.get());
    const Polygon &out =
        dynamic_cast<const Polygon &>(*corridor.back().shape.get());

    const Eigen::Vector3d p_w_gw = (in.position + out.position) * 0.5;
    const Eigen::Quaterniond q_wg = in.R_wb;

    Eigen::Vector3d relative_pos = q_wg.inverse() * (pos - p_w_gw);

    double x = relative_pos.x(), y = relative_pos.y(), z = relative_pos.z();

    double v0 = fabs(x), v1 = fabs(y), v2 = fmax(v0, v1), v3 = v2 - m_,
           v4 = fabs(v3), v5 = v4 - r_, v6 = fabs(z), v7 = v6 - d_,
           v8 = fmax(v5, v7), v9 = x > 0.0f ? 1.0f : x < 0.0f ? -1.0f : 0.0f,
           v10 = y > 0.0f ? 1.0f : y < 0.0f ? -1.0f : 0.0f, v11 = v0 - v1,
           v12 = v11 > 0.0f ? v0 : v1, v13 = v12 - v1,
           v14 = v13 > 0.0f ? v9 : 0.0f,
           v15 = v3 > 0.0f ? 1.0f : v3 < 0.0f ? -1.0f : 0.0f, v16 = v14 * v15,
           v17 = z > 0.0f ? 1.0f : z < 0.0f ? -1.0f : 0.0f, v18 = v5 - v7,
           v19 = v18 > 0.0f ? v5 : v7, v20 = v19 - v7,
           v21 = v20 > 0.0f ? v16 : 0.0f, v22 = v13 > 0.0f ? 0.0f : v10,
           v23 = v22 * v15, v24 = v20 > 0.0f ? v23 : 0.0f,
           v25 = v20 > 0.0f ? 0.0f : v17;

    const Eigen::Vector3d gradDistanceToRelativePos(v21, v24, v25);
    gradDistanceToPos = q_wg.toRotationMatrix() * gradDistanceToRelativePos;

    return v8;
  }

  void write(std::ofstream &os) override {
    const Polygon &in =
        dynamic_cast<const Polygon &>(*corridor.front().shape.get());
    const Polygon &out =
        dynamic_cast<const Polygon &>(*corridor.back().shape.get());
    Eigen::Vector3d position = (in.position + out.position) * 0.5;
    Eigen::Vector3d rpy = rad2deg(quaternionToEulerAnglesRPY(in.R_wb));
    os.precision(4);
    os << order << ":\n"
       << "  name: " << name << "\n"
       << "  shape: " << shapeName << "\n"
       << "  position: [" << position.x() << ", " << position.y() << ", "
       << position.z() << "]\n"
       << "  rpy: [" << rpy.x() << ", " << rpy.y() << ", " << rpy.z() << "]\n"
       << "  length: " << length << "\n"
       << "  midpoints: " << midpoints << "\n"
       << "  stationary: true\n\n";
    os.precision();
  }

  std::unordered_map<std::string, std::function<std::shared_ptr<Polygon>(
                                      const Yaml &yaml, const double drift)>>
      createShape{{"Triangle",
                   [](const Yaml &yaml, const double drift) {
                     return std::make_shared<Triangle>(yaml, drift);
                   }},
                  {"Rectangle",
                   [](const Yaml &yaml, const double drift) {
                     return std::make_shared<Rectangle>(yaml, drift);
                   }},
                  {"Pentagon",
                   [](const Yaml &yaml, const double drift) {
                     return std::make_shared<Pentagon>(yaml, drift);
                   }},
                  {"Hexagon", [](const Yaml &yaml, const double drift) {
                     return std::make_shared<Hexagon>(yaml, drift);
                   }}};
};

} // namespace drolib