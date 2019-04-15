/*
 * Copyright [2017] Max Planck Society. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
