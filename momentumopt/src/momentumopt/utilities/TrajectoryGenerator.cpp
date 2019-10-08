/**
 * @file TrajectoryGenerator.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#include <cmath>
#include <iostream>
#include <algorithm>
#include <momentumopt/utilities/TrajectoryGenerator.hpp>

namespace momentumopt
{
  // Interpolator
  void Interpolator::initialize(const InterpMat& pos_des, const InterpMat& vel_des, const InterpMat& acc_des)
  {
    cons_size_ = pos_des.rows() + vel_des.rows() + acc_des.rows();
	regression_vector_.resize(cons_size_);
	regression_matrix_.resize(cons_size_, cons_size_);
	regression_matrix_.setZero();

	time_span_.clear();
	time_span_.push_back(std::min(std::min(pos_des.col(0).minCoeff(), vel_des.col(0).minCoeff()), acc_des.col(0).minCoeff()));
	time_span_.push_back(std::max(std::max(pos_des.col(0).maxCoeff(), vel_des.col(0).maxCoeff()), acc_des.col(0).maxCoeff()));

	int row_start = 0;
	for (int row=0; row<pos_des.rows(); row++) {
	  double time = pos_des(row,0);
	  regression_vector_(row_start+row) = pos_des(row,1);
	  for (int col=0; col<cons_size_; col++)
		regression_matrix_(row_start+row,col) = std::pow(time,double(col));
	}

	row_start += pos_des.rows();
	for (int row=0; row<vel_des.rows(); row++) {
	  double time = vel_des(row,0);
	  regression_vector_(row_start+row) = vel_des(row,1);
	  for (int col=1; col<cons_size_; col++)
		regression_matrix_(row_start+row,col) = col*std::pow(time,col-1.0);
	}

	row_start += vel_des.rows();
	for (int row=0; row<acc_des.rows(); row++) {
	  double time = acc_des(row,0);
	  regression_vector_(row_start+row) = acc_des(row,1);
	  for (int col=2; col<cons_size_; col++)
		regression_matrix_(row_start+row,col) = col*(col-1)*std::pow(time,col-2.0);
	}

	regression_coeffs_ = regression_matrix_.householderQr().solve(regression_vector_);
  }

  void Interpolator::interpolate(const double time, double& position, double& velocity, double& acceleration)
  {
    double query_time = std::min(std::max(time, time_span_[0]), time_span_[1]);

    position = velocity = acceleration = 0.0;
    for (int i=0; i<cons_size_; i++) {
      position += regression_coeffs_(i)*std::pow(query_time, double(i));
      velocity += regression_coeffs_(i)*std::pow(query_time, i-1.0)*double(i);
      acceleration += regression_coeffs_(i)*std::pow(query_time, i-2.0)*double(i)*double(i-1.0);
    }
  }

  // TrajectoryGenerator
  void TrajectoryGenerator::initialize(const double tini, const Eigen::Vector3d& ini_position,
		                               const double tend, const Eigen::Vector3d& end_position,
                                       const Eigen::Vector3d& via_position)
  {
	Interpolator::InterpMat pos_mat, vel_mat, acc_mat;

	pos_mat.resize(3,2);
	vel_mat.resize(2,2);   vel_mat << tini, 0.0, tend, 0.0;
	acc_mat.resize(2,2);   acc_mat << tini, 0.0, tend, 0.0;

	for (int dir=0; dir<3; dir++) {
	  pos_mat << tini,            ini_position(dir),
                 0.5*(tini+tend), via_position(dir),
                 tend,            end_position(dir);
      traj_interp_[dir].initialize(pos_mat, vel_mat, acc_mat);
  	}
  }

  void TrajectoryGenerator::update(const double time)
  {
    for (int dir=0; dir<3; dir++)
      traj_interp_[dir].interpolate(time, pos_des_(dir), vel_des_(dir), acc_des_(dir));
  }
}
