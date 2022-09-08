// Copyright [2021] Optimus Ride Inc.

#pragma once

#include "altro/eigentypes.hpp"
#include "altro/problem/dynamics.hpp"

namespace altro {
namespace examples {

/**
 * @brief Simple kinematic model of a unicycle / differential drive robot
 *
 * Has 3 states and 2 controls.
 *
 * # Mathematical Formulation
 * \f[
 * \begin{bmatrix} \dot{x} \\ \dot{y} \dot{\theta}  \end{bmatrix} =
 * \begin{bmatrix} v \cos(\theta) \\ v \sin(\theta) \\ \omega \end{bmatrix}
 * \f]
 *
 * where
 * \f[
 * u = \begin{bmatrix} v \\ \omega \end{bmatrix}
 * \f]
 */
class Truck : public problem::ContinuousDynamics {
 public:
  Truck() = default;
  static constexpr int NStates = 4;
  static constexpr int NControls = 2;
  int StateDimension() const override { return NStates; }
  int ControlDimension() const override { return NControls; }

  void Evaluate(const VectorXdRef& x, const VectorXdRef& u, const float t,
                       Eigen::Ref<VectorXd> xdot) override;
  void Jacobian(const VectorXdRef& x, const VectorXdRef& u, const float t,
                Eigen::Ref<MatrixXd> jac) override;
  void Hessian(const VectorXdRef& x, const VectorXdRef& u, const float t, const VectorXdRef& b,
               Eigen::Ref<MatrixXd> hess) override;
  bool HasHessian() const override { return true; };
 private:
  const double l = 3;       // tractor length
  const double d = 5;       // trailer length
  const double dt = 0.1; // delta t
};

}  // namespace examples
}  // namespace altro