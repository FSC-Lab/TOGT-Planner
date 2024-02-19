#include "drolib/shape/polyhedron.h"

namespace drolib {

Polyhedron::Polyhedron(const Yaml &yaml) {
  load(yaml);
  initialize();
}  

Polyhedron::Polyhedron(const std::vector<Eigen::Vector3d> &vertices)
  : vertices(vertices) {
  this->name = "Polyhedron"; 
  this->shape = Shape::POLYHEDRON;
  initialize();
}

bool Polyhedron::initialize() {
  // Process data
  dim = vertices.size();
  if (dim < DIM3) {
    return false;;
  }

  position.setZero();
  std::for_each(vertices.begin(), vertices.end(), [&](const Eigen::Vector3d &v) {
    position += v;
  });
  position = position / dim;

  polyhedron.resize(DIM3, dim);
  for (int i = 0; i < dim; ++i) {
    polyhedron.col(i) = vertices[i];
  }

  coordinate.resize(DIM3, dim);
  coordinate.col(0) = polyhedron.col(0);
  coordinate.rightCols(dim - 1) = polyhedron.rightCols(dim - 1).colwise() - polyhedron.col(0);

  lbfgsParams.past = 0;
  lbfgsParams.delta = RES_TOLERANCE;
  lbfgsParams.g_epsilon = FLT_EPSILON;
  lbfgsParams.max_iterations = MAX_ITERATION;

  return true;
}

bool Polyhedron::load(const Yaml &yaml) {
  if (yaml.isNull()) return false;

  this->name = "Polyhedron";
  this->shape = Shape::POLYHEDRON;
  //TODO: load vertices

  return false;
}

//TODO:
bool Polyhedron::valid() const {
  return (dim == static_cast<int>(vertices.size()));
}

Eigen::VectorXd Polyhedron::toD(const Eigen::Vector3d &p) const {
  Eigen::Matrix3Xd ovPoly;
  ovPoly.resize(DIM3, dim + 1);
  ovPoly.col(0) = p; // the i-th points
  ovPoly.rightCols(dim) = coordinate;

  Eigen::VectorXd d(dim);
  d.setConstant(sqrt(1.0 / dim));

  double minSqrD;
  lbfgs_optimize(d, minSqrD, &Polyhedron::costTinyNLS, nullptr,
                        nullptr, &ovPoly, lbfgsParams);
  return d;
}

Eigen::Vector3d Polyhedron::toP(const Eigen::VectorXd &d) const {
    Eigen::VectorXd q = d.normalized().head(dim - 1);
    Eigen::Vector3d p =
        coordinate.rightCols(dim - 1) * q.cwiseProduct(q) + coordinate.col(0);
    return p;
  }

void Polyhedron::getGradD(const Eigen::VectorXd &d, const Eigen::Matrix3Xd &gradP,
                      const int i, const int j,
                      Eigen::Map<Eigen::VectorXd> &gradD) const {
  Eigen::VectorXd gradQ, unitQ;
  const double normInv = 1.0 / d.norm();
  unitQ = d * normInv;
  gradQ.resize(dim);
  gradQ.head(dim - 1) =
      (coordinate.rightCols(dim - 1).transpose() * gradP.col(i)).array() *
      unitQ.head(dim - 1).array() * 2.0;
  gradQ(dim - 1) = 0.0;
  gradD.segment(j, dim) = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;

  // Eigen::VectorXd gradQ, unitQ, gradDRes1, gradDRes2;
  // double normInv = 1.0 / d.norm();
  // unitQ = d * normInv;
  // gradQ.resize(dim);
  // gradQ.head(dim - 1) =
  //     (coordinate.rightCols(dim - 1).transpose() * gradP.col(i)).array() *
  //     unitQ.head(dim - 1).array() * 2.0;
  // gradQ.tail(1) << 0.0;
  // gradD.segment(j, dim) = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;
  // gradDRes1 = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;

  // std::cout << "gradDRes1: \n" << gradDRes1 <<std::endl;

  // const double normInv = 1.0 / d.norm(); // TODO: d * normInv and d.normalized() are different
  // gradD.segment(j, dim).head(dim - 1) = (coordinate.rightCols(dim - 1).transpose() * gradP.col(i)).array() * (d.normalized()).head(dim - 1).array() * 2.0;
  // gradD.segment(j, dim).tail(1) << 0.0;
  // gradD.segment(j, dim) = ( gradD.segment(j, dim) - (d.normalized()) * (d.normalized()).dot(gradD.segment(j, dim))) * normInv;


  // gradD.segment(j, dim) = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;
  // gradDRes2 = gradD.segment(j, dim);

  // gradD.segment(j, dim) = gradDRes2;
  // // std::cout << "gradDRes2: \n" << gradDRes2 <<std::endl;
  // if ((gradDRes2 - gradDRes1).squaredNorm() > 1.0e-6) {
  //   std::cout << "error: \n" << (gradDRes2 - gradDRes1).squaredNorm() <<std::endl;
  // }

  return;
}

double Polyhedron::costTinyNLS(void *ptr, const Eigen::VectorXd &d,
                                  Eigen::VectorXd &gradD) {
  const int n = d.size();
  const Eigen::Matrix3Xd &ovPoly = *(Eigen::Matrix3Xd *)ptr;

  const double sqrNormD = d.squaredNorm();
  const double invNormD = 1.0 / sqrt(sqrNormD);
  const Eigen::VectorXd unitD = d * invNormD;
  const Eigen::VectorXd r = unitD.head(n - 1);
  // find a point on the (n+1)-sphere, such that it can map to the 3D points
  // via the vertex vector
  const Eigen::Vector3d delta = ovPoly.rightCols(n - 1) * r.cwiseProduct(r) +
                                ovPoly.col(1) - ovPoly.col(0);

  double cost = delta.squaredNorm();
  gradD.head(n - 1) =
      (ovPoly.rightCols(n - 1).transpose() * (2 * delta)).array() *
      r.array() * 2.0;
  gradD(n - 1) = 0.0;
  gradD = (gradD - unitD.dot(gradD) * unitD).eval() * invNormD;

  const double sqrNormViolation = sqrNormD - 1.0;
  if (sqrNormViolation > 0.0) {
    double c = sqrNormViolation * sqrNormViolation;
    const double dc = 3.0 * c;
    c *= sqrNormViolation;
    cost += c;
    gradD += dc * 2.0 * d;
  }

  return cost;
}

}  // namespace drolib