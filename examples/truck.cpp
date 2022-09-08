// Copyright [2021] Optimus Ride Inc.

#include "examples/truck.hpp"

#include <cmath>

#include "altro/utils/utils.hpp"

namespace altro {
namespace examples {

void Truck::Evaluate(const VectorXdRef& x, const VectorXdRef& u, const float t,
                               Eigen::Ref<VectorXd> xdot) {
  ALTRO_UNUSED(t);
  double yaw = x(2);    // yaw angle
  double hitch = x(3);  // hitch angle
  double steer = u(0);  // steer angle
  double v = u(1);      // velocity
  xdot(0) = v * cos(yaw);
  xdot(1) = v * sin(yaw);
  xdot(2) = v / l * tan(steer);
  xdot(3) = v / d * sin(hitch - yaw);
  // xdot(4) = v * cos(hitch - yaw) * cos(hitch);
  // xdot(5) = v * cos(hitch - yaw) * sin(hitch);
}

void Truck::Jacobian(const VectorXdRef& x, const VectorXdRef& u, const float t,
                        Eigen::Ref<MatrixXd> jac) {
  ALTRO_UNUSED(t);
  double yaw = x(2);    // yaw angle
  double hitch = x(3);  // hitch angle
  double steer = u(0);  // steer angle
  double v = u(1);      // velocity

  jac(0, 2) = -v * sin(yaw);
  jac(0, 5) = cos(yaw);
  jac(1, 2) = v * cos(yaw);
  jac(1, 5) = sin(yaw);
  jac(2, 4) = v / (l * cos(steer) * cos(steer));
  jac(2, 5) = tan(steer) / l;
  jac(3, 2) = -v / d * cos(hitch - yaw);
  jac(3, 3) = v / d * cos(hitch - yaw);
  jac(3, 5) = sin(hitch - yaw) / d;
  // jac(0, 2) = -v * sin(yaw);
  // jac(0, 7) = cos(yaw);
  // jac(1, 2) = v * cos(yaw);
  // jac(1, 7) = sin(yaw);
  // jac(2, 6) = v / (l * cos(steer) * cos(steer));
  // jac(2, 7) = tan(steer) / l;
  // jac(3, 2) = -v / d * cos(hitch - yaw);
  // jac(3, 3) = v / d * cos(hitch - yaw);
  // jac(3, 7) = sin(hitch - yaw) / d;
  // jac(4, 2) = v * sin(hitch - yaw) * cos(hitch);
  // jac(4, 3) = v * (-sin(hitch - yaw) * cos(hitch) - cos(hitch - yaw) * sin(hitch));
  // jac(4, 7) = cos(hitch - yaw) * cos(hitch);
  // jac(5, 2) = v * sin(hitch - yaw) * sin(hitch);
  // jac(5, 3) = v * (-sin(hitch - yaw) * sin(hitch) + cos(hitch - yaw) * cos(hitch));
  // jac(5, 7) = cos(hitch - yaw) * sin(hitch);
}

void Truck::Hessian(const VectorXdRef& x, const VectorXdRef& u, const float t,
                       const VectorXdRef& b, Eigen::Ref<MatrixXd> hess) {
  ALTRO_UNUSED(t);
  double theta = x(2);  // angle
  double v = u(1);      // linear velocity
  hess(2, 2) = -b(0) * v * cos(theta) - b(1) * v * sin(theta);
  hess(2, 3) = -b(0) * sin(theta) + b(1) * cos(theta);
  hess(3, 2) = hess(2, 3);
}

}  // namespace examples
}  // namespace altro