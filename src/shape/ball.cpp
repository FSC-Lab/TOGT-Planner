#include "drolib/shape/ball.h"

namespace drolib {

Ball::Ball() {
  this->name = "Ball";
}

Ball::Ball(const Yaml &yaml) {
  load(yaml);
  initialize();
}  

bool Ball::initialize() {
  dim = DIM3;
  return true;
}

bool Ball::load(const Yaml &yaml) {
  if (yaml.isNull()) return false;

  this->name = "Ball";
  this->shape = Shape::BALL;
  yaml["position"].getIfDefined(position);
  yaml["radius"].getIfDefined(radius);
  yaml["margin"].getIfDefined(margin);
  return true;
}

//TODO:
bool Ball::valid() const {
  return (radius > margin) && (dim == DIM3);
}

Eigen::VectorXd Ball::toD(const Eigen::Vector3d &p) const {
  Eigen::VectorXd d(dim);
  Eigen::Vector3d dist = p - position;
  const double sqrNormDist = dist.squaredNorm();
  const double realRadius = radius - margin * 0.5;
  const double delta = realRadius * realRadius - sqrNormDist;

  double scale;
  if (delta < 1e-4 || sqrNormDist < 1e-4) {
    scale = 1.0 / (2 * realRadius);
  } else {
    scale = (realRadius - sqrt(delta)) / sqrNormDist;
  }

  d = scale * dist;

  return d;
}

Eigen::Vector3d Ball::toP(const Eigen::VectorXd &d) const {
  Eigen::Vector3d p;
  const double realRadius = radius - margin * 0.5;
  p = position + 2 * realRadius * d / (d.squaredNorm() + 1);
  return p;
}

void Ball::getGradD(const Eigen::VectorXd &d,
                              const Eigen::Matrix3Xd &gradP, const int i, const int j,
                              Eigen::Map<Eigen::VectorXd> &gradD) const {
  //TODO:
  const double sqrNormDl = d.squaredNorm() + 1;
  const double realRadius = radius - margin * 0.5;
  const double invSqrNormDl = 1.0 / sqrNormDl;
  gradD.segment(j, dim) = 2 * realRadius * invSqrNormDl * gradP.col(i) - 4 * realRadius * d.dot(gradP.col(i)) * invSqrNormDl * invSqrNormDl * d;
  
  // const double sqrNormDl = d.squaredNorm() + 1;
  // const double invSqrNormDl = 1.0 / sqrNormDl;
  // const double realRadius = radius - margin * 0.5;
  // Eigen::VectorXd gradQ;
  // gradQ.resize(dim);
  // gradQ = 2 * realRadius * invSqrNormDl * gradP.col(i) -
  //         4 * realRadius * d.dot(gradP.col(i)) * invSqrNormDl * invSqrNormDl * d;
  // gradD.segment(j, dim) = gradQ;

  
  return;
}


}  // namespace drolib