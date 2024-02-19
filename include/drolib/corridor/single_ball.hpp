#pragma once

#include "drolib/shape/ball.h"
#include "drolib/base/corridor_base.hpp"

namespace drolib {

class SingleBall : public CorridorBase {
public:
  SingleBall();

  SingleBall(const Yaml& yaml, const std::string& order = "");

  void write(std::ofstream& os) override {
    const Ball &ball = dynamic_cast<const Ball &>(*corridor.front().shape);
    os.precision(4);
    os << order << ":\n"
       << "  name: " << name << "\n"
       << "  type: " << "'SingleBall'\n"
       << "  position: [" << ball.position.x() << ", " << ball.position.y() << ", " << ball.position.z() << "]\n"
       << "  radius: " << ball.radius << "\n"
       << "  margin: " << ball.margin << "\n"
       << "  stationary: true\n\n";
    os.precision();
  }       
};

} // namespace drolib