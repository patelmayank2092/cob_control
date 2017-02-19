/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_twist_controller
 *
 * \author
 *   Author: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *
 * \date Date of creation: March, 2015
 *
 * \brief
 *   Implementation of an JLA solver.
 *   Special constraint: Avoid joint limits.
 *
 ****************************************************************/

#include "cob_twist_controller/constraint_solvers/solvers/non_linear_least_square_solver.h"

/**
 * Specific implementation of the solve method using a weighted least norm.
 * This is done by calculation of a weighting which is dependent on inherited classes for the Jacobian.
 * Uses the base implementation of calculatePinvJacobianBySVD to calculate the pseudo-inverse (weighted) Jacobian.
 */
struct CostFunctor {
  template <typename T> bool operator()(const T* const x, T* residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};

Eigen::MatrixXd NonLinearLeastSquareNormSolver::solve(const Vector6d_t& in_cart_velocities,
                                               const JointStates& joint_states)
{
    // The variable to solve for with its initial value. It will be
      // mutated in place by the solver.
      double x = 0.5;
      const double initial_x = x;
      // Build the problem.
      Problem problem;
      // Set up the only cost function (also known as residual). This uses
      // auto-differentiation to obtain the derivative (jacobian).
      CostFunction* cost_function = new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
      problem.AddResidualBlock(cost_function, NULL, &x);
      // Run the solver!
      Solver::Options options;
      options.minimizer_progress_to_stdout = true;
      Solver::Summary summary;
      Solve(options, &problem, &summary);
      std::cout << summary.BriefReport() << "\n";
      std::cout << "x : " << initial_x << " -> " << x << "\n";
}

/**
 * This function returns the identity as weighting matrix for base functionality.
 */
Eigen::MatrixXd NonLinearLeastSquareNormSolver::calculateWeighting(const JointStates& joint_states) const
{
    uint32_t cols = this->jacobian_data_.cols();
    Eigen::VectorXd weighting = Eigen::VectorXd::Ones(cols);
    return weighting.asDiagonal();
}
