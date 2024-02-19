#include "drolib/corridor/single_ball.hpp"

namespace drolib {

SingleBall::SingleBall() {
  this->type = "SingleBall";
}

SingleBall::SingleBall(const Yaml &yaml, const std::string &order) {
  this->type = "SingleBall";
  this->order = order;
  yaml["name"].getIfDefined(name);
  corridor.emplace_back(std::make_shared<Ball>(yaml));
}

} // namespace drolib