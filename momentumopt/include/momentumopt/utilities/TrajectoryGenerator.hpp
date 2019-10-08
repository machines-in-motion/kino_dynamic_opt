/**
 * @file TrajectoryGenerator.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#pragma once

#include <vector>
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

namespace momentumopt
{
  enum class InterpolationMethod {
    PiecewiseLinear,
    PiecewiseConstant,
  };

  // Class to generate trajectories around way-points in position, velocity and acceleration
  class Interpolator
  {
    public:
	  typedef Eigen::Matrix<double,Eigen::Dynamic,2> InterpMat;

    public:
	  Interpolator(){}
	  ~Interpolator(){}
      void initialize(const InterpMat& pos_des, const InterpMat& vel_des, const InterpMat& acc_des);
      void interpolate(const double time, double& position, double& velocity, double& acceleration);

    private:
      double cons_size_;
	  std::vector<double> time_span_;
	  Eigen::MatrixXd regression_matrix_;
	  Eigen::VectorXd regression_vector_, regression_coeffs_;
  };

  // Class to generate 3d continuous and smooth trajectories
  class TrajectoryGenerator
  {
    public:
	  TrajectoryGenerator(){};
      ~TrajectoryGenerator(){};
      void initialize(const double tini, const Eigen::Vector3d& ini_position,
                      const double tend, const Eigen::Vector3d& end_position,
                      const Eigen::Vector3d& via_position);
      void update(const double time);

	  const Eigen::Vector3d& desPos() const { return pos_des_; }
	  const Eigen::Vector3d& desVel() const { return vel_des_; }
	  const Eigen::Vector3d& desAcc() const { return acc_des_; }

    private:
	  Interpolator traj_interp_[3];
	  Eigen::Vector3d pos_des_, vel_des_, acc_des_;
  };
}
