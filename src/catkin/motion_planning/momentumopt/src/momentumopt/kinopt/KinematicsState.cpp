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

#include <momentumopt/kinopt/KinematicsState.hpp>

namespace momentumopt {

  // RobotPosture class functions implementation
  std::string RobotPosture::toString() const
  {
	std::stringstream text;
    text << "    base pos     " << this->basePosition().transpose() << "\n";
    text << "    base ori     " << this->baseOrientation().coeffs().transpose() << "\n";
    text << "    joints pos   " << this->jointPositions().transpose() << "\n";
    return text.str();
  }

  // RobotVelocity class functions implementation
  std::string RobotVelocity::toString() const
  {
	std::stringstream text;
    text << "    base lin vel " << this->baseLinearVelocity().transpose() << "\n";
    text << "    base ang vel " << this->baseAngularVelocity().transpose() << "\n";
    text << "    joints vel   " << this->jointVelocities().transpose() << "\n";
    return text.str();
  }

  // RobotAcceleration class functions implementation
  std::string RobotAcceleration::toString() const
  {
	std::stringstream text;
    text << "    base lin acc " << this->baseLinearAcceleration().transpose() << "\n";
    text << "    base ang acc " << this->baseAngularAcceleration().transpose() << "\n";
    text << "    joints acc   " << this->jointAccelerations().transpose() << "\n";
    return text.str();
  }

  // KinematicsState class functions implementation
  KinematicsState::KinematicsState(int num_joints)
    : robot_posture_(num_joints),
      robot_velocity_(num_joints),
      robot_acceleration_(num_joints)
  {
    com_.setZero();
    lmom_.setZero();
    amom_.setZero();

    for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
      eff_positions_[eff_id] = Eigen::Vector3d::Zero();
      eff_velocities_[eff_id] = Eigen::Vector3d::Zero();
      eff_accelerations_[eff_id] = Eigen::Vector3d::Zero();
      eff_orientations_[eff_id] = Eigen::Quaternion<double>::Identity();
    }
  }

  std::string KinematicsState::toString() const
  {
	std::stringstream text;
    text << "  com     " << this->centerOfMass().transpose() << "\n";
    text << "  lmom    " << this->linearMomentum().transpose() << "\n";
    text << "  amom    " << this->angularMomentum().transpose() << "\n";

	for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++)
	{
	  text << "  eff_id " << eff_id << "\n";
      text << "    pos  " << this->endeffectorPosition(eff_id).transpose() << "\n";
      text << "    vel  " << this->endeffectorVelocity(eff_id).transpose() << "\n";
      text << "    acc  " << this->endeffectorAcceleration(eff_id).transpose() << "\n";
      text << "    ori  " << this->endeffectorOrientation(eff_id).coeffs().transpose() << "\n";
	}

    text << "  jnt_pos  \n" << this->robotPosture() << "\n";
    text << "  jnt_vel  \n" << this->robotVelocity() << "\n";
    text << "  jnt_acc  \n" << this->robotAcceleration() << "\n";

	return text.str();
  }

  // KinematicsSequence class function implementations
  void KinematicsSequence::resize(int num_timesteps, int num_joints)
  {
    kinematics_sequence_.clear();
    for (int time_id=0; time_id<num_timesteps; time_id++)
      kinematics_sequence_.push_back(KinematicsState(num_joints));
  }

  std::string KinematicsSequence::toString() const
  {
    std::stringstream text;
    for (int time_id=0; time_id<this->size(); time_id++) {
      text << "time " << time_id << "\n";
      text << kinematics_sequence_[time_id];
    }
	return text.str();
  }

}
