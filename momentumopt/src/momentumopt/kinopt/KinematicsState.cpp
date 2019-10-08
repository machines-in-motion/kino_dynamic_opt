/**
 * @file KinematicsState.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
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
      robot_acceleration_(num_joints),
      num_joints_(num_joints)
  {
    com_.setZero();
    lmom_.setZero();
    amom_.setZero();

    eff_positions_.clear();
    eff_velocities_.clear();
    eff_orientations_.clear();
    eff_accelerations_.clear();
    for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
      eff_positions_.push_back(Eigen::Vector3d::Zero());
      eff_velocities_.push_back(Eigen::Vector3d::Zero());
      eff_accelerations_.push_back(Eigen::Vector3d::Zero());
      eff_orientations_.push_back(Eigen::Quaternion<double>::Identity());
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
      //text << "    vel  " << this->endeffectorVelocity(eff_id).transpose() << "\n";
      //text << "    acc  " << this->endeffectorAcceleration(eff_id).transpose() << "\n";
      //text << "    ori  " << this->endeffectorOrientation(eff_id).coeffs().transpose() << "\n";
	}

    text << "  jnt_pos  \n" << this->robotPosture() << "\n";
    //text << "  jnt_vel  \n" << this->robotVelocity() << "\n";
    //text << "  jnt_acc  \n" << this->robotAcceleration() << "\n";

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
