// Copyright [2021] Optimus Ride Inc.

#pragma once

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "altro/eigentypes.hpp"
#include "altro/problem/problem.hpp"

#include "altro/augmented_lagrangian/al_solver.hpp"
#include "altro/augmented_lagrangian/al_problem.hpp"
#include "altro/common/trajectory.hpp"
#include "altro/ilqr/ilqr.hpp"
#include "altro/problem/discretized_model.hpp"
#include "examples/basic_constraints.hpp"
#include "examples/obstacle_constraints.hpp"
#include "examples/quadratic_cost.hpp"
#include "examples/truck.hpp"

namespace altro {
namespace problems {

class TruckProblem {
 public:
  static constexpr int NStates = 4;
  static constexpr int NControls = 2;

  TruckProblem();

  enum Scenario { kTurn90, kThreeObstacles, StraightRoadWithObs, Uturn, LaneChange, ForwardObs, Largecurve };

	using ModelType = altro::problem::DiscretizedModel<altro::examples::Truck>;
	using CostFunType = altro::examples::QuadraticCost;

  // Problem Data
  static constexpr int HEAP = Eigen::Dynamic;
  const int n = NStates;
  const int m = NControls;

  int N = 50;
  ModelType model = ModelType(altro::examples::Truck());

  Eigen::MatrixXd Q = Eigen::VectorXd::Constant(NStates, 1e-2).asDiagonal();
  Eigen::MatrixXd R = Eigen::VectorXd::Constant(NControls, 1e-2).asDiagonal();
  Eigen::MatrixXd Qf = Eigen::VectorXd::Constant(NStates, 100).asDiagonal();
  Eigen::Vector4d xf = Eigen::Vector4d(0, 0, 0, 0);
  Eigen::Vector4d x0 = Eigen::Vector4d(0, 0, 0, 0);
  Eigen::VectorXd u0 = Eigen::VectorXd::Constant(NControls, 0.0);
  Eigen::VectorXd uref = Eigen::VectorXd::Constant(NControls, 0.0);
  std::shared_ptr<examples::QuadraticCost> qcost;
  std::shared_ptr<examples::QuadraticCost> qterm;

  double v_bnd = 11;  // linear velocity bound = 22 m/s
  double w_bnd = 1.5;  // angular velocity bound
  // double steer_bnd = 0.174532925;  // steer angle bound = 10 deg
  double steer_bnd = 0.523598776;  // steer angle bound = 30 deg
  Eigen::VectorXd cx;  // x-coordinates of obstacles
  Eigen::VectorXd cy;  // y-coordinates of obstacles
  Eigen::VectorXd cr;  // radii of obstacles
  std::vector<double> lb;
  std::vector<double> ub;
  altro::examples::CircleConstraint obstacles;

//   altro::problem::Problem MakeProblem(const bool add_constraints = true, Eigen::VectorXd x_init  = Eigen::VectorXd(NStates, 0));
  altro::problem::Problem MakeProblem(const bool add_constraints = true, Eigen::Vector4d x_init  = Eigen::Vector4d(0, 0, 0, 0));

  template <int n_size = NStates, int m_size = NControls>
  altro::Trajectory<n_size, m_size> InitialTrajectory();

  template <int n_size = NStates, int m_size = NControls>
  altro::ilqr::iLQR<n_size, m_size> MakeSolver(const bool alcost = false);

  template <int n_size = NStates, int m_size = NControls>
//   altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size> MakeALSolver(Eigen::VectorXd x_init = Eigen::VectorXd(NStates, 0));
  altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size> MakeALSolver(Eigen::Vector4d x_init = Eigen::Vector4d(0, 0, 0, 0));

  void SetScenario(Scenario scenario) { scenario_ = scenario; }

  float GetTimeStep() const { return tf / N; }

 private:
  Scenario scenario_ = StraightRoadWithObs;
  float tf = 3.0;
};

template <int n_size, int m_size>
altro::Trajectory<n_size, m_size> TruckProblem::InitialTrajectory() {
  altro::Trajectory<n_size, m_size> Z(n, m, N);
  for (int k = 0; k < N; ++k) {
    Z.Control(k) = u0;
  }
  float h = GetTimeStep(); 
  Z.SetUniformStep(h);
  return Z;
}

template <int n_size, int m_size>
altro::ilqr::iLQR<n_size, m_size> TruckProblem::MakeSolver(const bool alcost) {
  altro::problem::Problem prob = MakeProblem();
  if (alcost) {
    prob = altro::augmented_lagrangian::BuildAugLagProblem<n_size, m_size>(prob);
  }
  altro::ilqr::iLQR<n_size, m_size> solver(prob);

  std::shared_ptr<altro::Trajectory<n_size, m_size>> traj_ptr =
      std::make_shared<altro::Trajectory<n_size, m_size>>(InitialTrajectory<n_size, m_size>());

  solver.SetTrajectory(traj_ptr);
  solver.Rollout();
  return solver;
}

template <int n_size, int m_size>
altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size>
TruckProblem::MakeALSolver(Eigen::Vector4d x_init) {
  altro::problem::Problem prob = MakeProblem(true, x_init);
  altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size> solver_al(prob);
  solver_al.SetTrajectory(
      std::make_shared<altro::Trajectory<NStates, NControls>>(InitialTrajectory()));
  solver_al.GetiLQRSolver().Rollout();

  return solver_al;
}

}  // namespace problems
}  // namespace altro