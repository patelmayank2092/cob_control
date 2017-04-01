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
class Q1{
  public:

    Q1(Matrix J, Vector x, double lambda): J_(J), x_(x), lambda_(lambda) {}

    template <typename T>
    bool operator()(const T* const q, T* e) const {
        e[0] = T(x_(0))-T(J_(0,0))*q[0]-T(J_(0,1))*q[1]-T(J_(0,2))*q[2]-T(J_(0,3))*q[3]-T(J_(0,4))*q[4]-T(J_(0,5))*q[5];
        return true;
    }

  private:
    Eigen::MatrixXd J_;
    Vector6d_t x_;
    double lambda_;
};

class Q2{
  public:

    Q2(Matrix J, Vector x, double lambda): J_(J), x_(x), lambda_(lambda) {}

    template <typename T>
    bool operator()(const T* const q, T* e) const {
        e[0] = T(x_(1))-T(J_(1,0))*q[0]-T(J_(1,1))*q[1]-T(J_(1,2))*q[2]-T(J_(1,3))*q[3]-T(J_(1,4))*q[4]-T(J_(1,5))*q[5];
        return true;
    }

  private:
    Eigen::MatrixXd J_;
    Vector6d_t x_;
    double lambda_;
};
class Q3{
  public:

    Q3(Matrix J, Vector x, double lambda): J_(J), x_(x), lambda_(lambda){}

    template <typename T>
    bool operator()(const T* const q, T* e) const {
        e[0] = T(x_(2))-T(J_(2,0))*q[0]-T(J_(2,1))*q[1]-T(J_(2,2))*q[2]-T(J_(2,3))*q[3]-T(J_(2,4))*q[4]-T(J_(2,5))*q[5];
        return true;
    }

  private:
    Eigen::MatrixXd J_;
    Vector6d_t x_;
    double lambda_;
};
class Q4{
  public:

    Q4(Matrix J, Vector x, double lambda): J_(J), x_(x), lambda_(lambda) {}

    template <typename T>
    bool operator()(const T* const q, T* e) const {
        e[0] = T(x_(3))-T(J_(3,0))*q[0]-T(J_(3,1))*q[1]-T(J_(3,2))*q[2]-T(J_(3,3))*q[3]-T(J_(3,4))*q[4]-T(J_(3,5))*q[5];
        return true;
    }

  private:
    Eigen::MatrixXd J_;
    Vector6d_t x_;
    double lambda_;
};
class Q5{
  public:

    Q5(Matrix J, Vector x, double lambda): J_(J), x_(x), lambda_(lambda) {}

    template <typename T>
    bool operator()(const T* const q, T* e) const {
        e[0] = T(x_(4))-T(J_(4,0))*q[0]-T(J_(4,1))*q[1]-T(J_(4,2))*q[2]-T(J_(4,3))*q[3]-T(J_(4,4))*q[4]-T(J_(4,5))*q[5];
        return true;
    }

  private:
    Eigen::MatrixXd J_;
    Vector6d_t x_;
    double lambda_;
};
class Q6{
  public:

    Q6(Matrix J, Vector x, double lambda): J_(J), x_(x), lambda_(lambda) {}

    template <typename T>
    bool operator()(const T* const q, T* e) const {
        e[0] = T(x_(5))-T(J_(5,0))*q[0]-T(J_(5,1))*q[1]-T(J_(5,2))*q[2]-T(J_(5,3))*q[3]-T(J_(5,4))*q[4]-T(J_(5,5))*q[5];
        return true;
    }

  private:
    Eigen::MatrixXd J_;
    Vector6d_t x_;
    double lambda_;
};


Eigen::MatrixXd NonLinearLeastSquareNormSolver::solve(const Vector6d_t& in_cart_velocities,
                                               const JointStates& joint_states)
{
    // The variable to solve for with its initial value. It will be
      // mutated in place by the solver.
      double x[7];
      double initial_x[7];

      for( int i=0; i<7;i++){
          initial_x[i]=joint_states.current_q_dot_(i);
          x[i]=initial_x[i];
      }
      Eigen::MatrixXd Jinv = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
      Eigen::VectorXd q = this->jacobian_data_ * joint_states.current_q_dot_.data;

      // Build the problem.
      Problem problem;
      // Set up the only cost function (also known as residual). This uses
      // auto-differentiation to obtain the derivative (jacobian).
      double damping=0.01;
      CostFunction* cost_function = new AutoDiffCostFunction<Q1, 1, 6>(new Q1(this->jacobian_data_,in_cart_velocities,damping));
      CostFunction* cost_function2 = new AutoDiffCostFunction<Q2, 1, 6>(new Q2(this->jacobian_data_,in_cart_velocities,damping));
      CostFunction* cost_function3 = new AutoDiffCostFunction<Q3, 1, 6>(new Q3(this->jacobian_data_,in_cart_velocities,damping));
      CostFunction* cost_function4 = new AutoDiffCostFunction<Q4, 1, 6>(new Q4(this->jacobian_data_,in_cart_velocities,damping));
      CostFunction* cost_function5 = new AutoDiffCostFunction<Q5, 1, 6>(new Q5(this->jacobian_data_,in_cart_velocities,damping));
      CostFunction* cost_function6 = new AutoDiffCostFunction<Q6, 1, 6>(new Q6(this->jacobian_data_,in_cart_velocities,damping));
      problem.AddResidualBlock(cost_function,NULL,x);
      problem.AddResidualBlock(cost_function2,NULL,x);
      problem.AddResidualBlock(cost_function3,NULL,x);
      problem.AddResidualBlock(cost_function4,NULL,x);
      problem.AddResidualBlock(cost_function5,NULL,x);
      problem.AddResidualBlock(cost_function6,NULL,x);
      // Run the solver!
      /*Solver::Options options;
      options.minimizer_progress_to_stdout = true;
      Solver::Summary summary;

      Solve(options, &problem, &summary);
      std::cout << summary.BriefReport() << "\n";*/
      Solver::Options options;
      Solver::Summary summary;
      options.minimizer_progress_to_stdout = false;
      Solve(options, &problem, &summary);
      Eigen::MatrixXd qdots_out(7,1);

      for( int i=0; i<6;i++){
          qdots_out(i)=x[i];
      }

      return qdots_out;
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
