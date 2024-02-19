#pragma once

#include "drolib/shape/rectangle.h"
#include "drolib/shape/triangle.h"
#include "drolib/shape/pentagon.h"
#include "drolib/shape/hexagon.h"
#include "drolib/base/corridor_base.hpp"
#include <unordered_map>

namespace drolib {

class PrismaCorridor : public CorridorBase {
public:
  std::string shapeName;
  double length{0.0};
  int midpoints{0};

  PrismaCorridor();

  PrismaCorridor(const std::string shape, const Yaml& yaml, const std::string& order = "");
  
  void write(std::ofstream& os) override {
    const Polygon &in = dynamic_cast<const Polygon &>(*corridor.front().shape.get());
    const Polygon &out = dynamic_cast<const Polygon &>(*corridor.back().shape.get());
    Eigen::Vector3d position = (in.position+ out.position) * 0.5;
    Eigen::Vector3d rpy = rad2deg(quaternionToEulerAnglesRPY(in.R_wb));
    os.precision(4);
    os << order << ":\n"
       << "  name: " << name << "\n"
       << "  shape: " << shapeName << "\n"
       << "  position: [" << position.x() << ", " << position.y() << ", " << position.z() << "]\n"
       << "  rpy: [" << rpy.x() << ", " << rpy.y() << ", " << rpy.z() << "]\n"
       << "  length: " << length << "\n"
       << "  midpoints: " << midpoints << "\n"
       << "  stationary: true\n\n";
    os.precision();
  }   

  std::unordered_map<std::string, std::function<std::shared_ptr<Polygon>(const Yaml &yaml, const double drift)>>
    createShape{
        {"Triangle",
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
        {"Hexagon",
          [](const Yaml &yaml, const double drift) {
            return std::make_shared<Hexagon>(yaml, drift);
          }}};

};

} // namespace drolib