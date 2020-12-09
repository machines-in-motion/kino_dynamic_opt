/**
 * @file KinematicsState.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#pragma once

#include <array>
#include <vector>
#include <memory>
#include <iostream>
#include <Eigen/Eigen>

#include <momentumopt/setting/Definitions.hpp>

namespace momentumopt {

  /*! container for robot position variables */
  class RobotPosture
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
	  RobotPosture(int num_joints)
        : num_joints_(num_joints),
          base_position_(Eigen::Vector3d::Zero()),
          joint_positions_(Eigen::VectorXd(num_joints).setZero()),
		  base_orientation_(Eigen::Quaternion<double>::Identity())
      {}
	  ~RobotPosture(){}

      Eigen::Vector3d& basePosition() { return base_position_; }
      Eigen::VectorXd& jointPositions() { return joint_positions_; }
      Eigen::Quaternion<double>& baseOrientation() { return base_orientation_; }

      const Eigen::Vector3d& basePosition() const { return base_position_; }
      const Eigen::VectorXd& jointPositions() const { return joint_positions_; }
      const Eigen::Quaternion<double>& baseOrientation() const { return base_orientation_; }
      const Eigen::Vector4d pyBaseOrientation() const { return Eigen::Vector4d(base_orientation_.x(), base_orientation_.y(), base_orientation_.z(), base_orientation_.w()); }

      void basePosition(const Eigen::Vector3d& base_position) { base_position_ = base_position; }
      void jointPositions(const Eigen::VectorXd& joint_positions) { joint_positions_ = joint_positions; }
      void baseOrientation(const Eigen::Quaternion<double>& base_orientation) { base_orientation_ = base_orientation; }
      void pyBaseOrientation(const Eigen::Vector4d base_orientation) { base_orientation_ = Eigen::Quaternion<double>(base_orientation[3], base_orientation[0], base_orientation[1], base_orientation[2]); }

      Eigen::VectorXd generalizedJointPositions() const
      {
        Eigen::VectorXd generalized_joint_positions(num_joints_+7);
        generalized_joint_positions.head(3) = base_position_;
        generalized_joint_positions.tail(num_joints_) = joint_positions_;
        generalized_joint_positions.segment<4>(3) = Eigen::Vector4d(base_orientation_.x(), base_orientation_.y(), base_orientation_.z(), base_orientation_.w());
        return generalized_joint_positions;
      }

	  std::string toString() const;
	  friend std::ostream& operator<<(std::ostream &os, const RobotPosture& obj) { return os << obj.toString(); }

    private:
      int num_joints_;
      Eigen::Vector3d base_position_;
      Eigen::VectorXd joint_positions_;
      Eigen::Quaternion<double> base_orientation_;
  };

  /*! container for robot velocity variables */
  class RobotVelocity
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
	  RobotVelocity(int num_joints)
        : num_joints_(num_joints),
		  generalized_joint_velocities_(Eigen::VectorXd(num_joints+6).setZero())
      {}
	  ~RobotVelocity(){}

      Eigen::VectorXd& generalizedJointVelocities() { return generalized_joint_velocities_; }
      Eigen::Ref<Eigen::Vector3d> baseLinearVelocity() { return generalized_joint_velocities_.segment<3>(0); }
      Eigen::Ref<Eigen::Vector3d> baseAngularVelocity() { return generalized_joint_velocities_.segment<3>(3); }
      Eigen::Ref<Eigen::VectorXd> jointVelocities() { return generalized_joint_velocities_.tail(num_joints_); }

      const Eigen::VectorXd& generalizedJointVelocities() const { return generalized_joint_velocities_; }
      const Eigen::Ref<const Eigen::Vector3d> baseLinearVelocity() const { return generalized_joint_velocities_.segment<3>(0); }
      const Eigen::Ref<const Eigen::Vector3d> baseAngularVelocity() const { return generalized_joint_velocities_.segment<3>(3); }
      const Eigen::Ref<const Eigen::VectorXd> jointVelocities() const { return generalized_joint_velocities_.tail(num_joints_); }

      void generalizedJointVelocities(const Eigen::VectorXd& generalized_joint_velocities) { generalized_joint_velocities_ = generalized_joint_velocities; }
      void jointVelocities(const Eigen::Ref<const Eigen::VectorXd> joint_velocities) { generalized_joint_velocities_.tail(num_joints_) = joint_velocities; }
      void baseLinearVelocity(const Eigen::Ref< const Eigen::Vector3d> base_linear_velocity) { generalized_joint_velocities_.segment<3>(0) = base_linear_velocity; }
      void baseAngularVelocity(const Eigen::Ref<const Eigen::Vector3d> base_angular_velocity) { generalized_joint_velocities_.segment<3>(3) = base_angular_velocity; }

	  std::string toString() const;
	  friend std::ostream& operator<<(std::ostream &os, const RobotVelocity& obj) { return os << obj.toString(); }

    private:
      int num_joints_;
      Eigen::VectorXd generalized_joint_velocities_;
  };

  /*! container for robot acceleration variables */
  class RobotAcceleration
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
	  RobotAcceleration(int num_joints)
        : num_joints_(num_joints),
          generalized_joint_accelerations_(Eigen::VectorXd(num_joints+6).setZero())
      {}
	  ~RobotAcceleration(){}

      Eigen::VectorXd& generalizedJointAccelerations() { return generalized_joint_accelerations_; }
      Eigen::Ref<Eigen::Vector3d> baseLinearAcceleration() { return generalized_joint_accelerations_.segment<3>(0); }
      Eigen::Ref<Eigen::Vector3d> baseAngularAcceleration() { return generalized_joint_accelerations_.segment<3>(3); }
      Eigen::Ref<Eigen::VectorXd> jointAccelerations() { return generalized_joint_accelerations_.tail(num_joints_); }

      const Eigen::VectorXd& generalizedJointAccelerations() const { return generalized_joint_accelerations_; }
      const Eigen::Ref<const Eigen::Vector3d> baseLinearAcceleration() const { return generalized_joint_accelerations_.segment<3>(0); }
      const Eigen::Ref<const Eigen::Vector3d> baseAngularAcceleration() const { return generalized_joint_accelerations_.segment<3>(3); }
      const Eigen::Ref<const Eigen::VectorXd> jointAccelerations() const { return generalized_joint_accelerations_.tail(num_joints_); }

      void generalizedJointAccelerations(const Eigen::VectorXd& generalized_joint_accelerations) { generalized_joint_accelerations_ = generalized_joint_accelerations; }
      void jointAccelerations(const Eigen::Ref<const Eigen::VectorXd> joint_accelerations) { generalized_joint_accelerations_.tail(num_joints_) = joint_accelerations; }
      void baseLinearAcceleration(const Eigen::Ref< const Eigen::Vector3d> base_linear_acceleration) { generalized_joint_accelerations_.segment<3>(0) = base_linear_acceleration; }
      void baseAngularAcceleration(const Eigen::Ref<const Eigen::Vector3d> base_angular_acceleration) { generalized_joint_accelerations_.segment<3>(3) = base_angular_acceleration; }

	  std::string toString() const;
	  friend std::ostream& operator<<(std::ostream &os, const RobotAcceleration& obj) { return os << obj.toString(); }

    private:
      int num_joints_;
      Eigen::VectorXd generalized_joint_accelerations_;
  };

  /**
   * This class is a container for all variables required to define a
   * kinematics state: joint posture, joint velocities and accelerations,
   * endeffector poses and momentum.
   */
  class KinematicsState
  {
    public:
      KinematicsState(int num_joints);
      ~KinematicsState(){}

      // center of mass and momentum
	  Eigen::Vector3d& centerOfMass() { return com_; }
	  Eigen::Vector3d& linearMomentum() { return lmom_; }
	  Eigen::Vector3d& angularMomentum() { return amom_; }

	  const Eigen::Vector3d& centerOfMass() const { return com_; }
	  const Eigen::Vector3d& linearMomentum() const { return lmom_; }
	  const Eigen::Vector3d& angularMomentum() const { return amom_; }

	  void centerOfMass(const Eigen::Vector3d& com) { com_ = com; }
	  void linearMomentum(const Eigen::Vector3d& lmom) { lmom_ = lmom; }
	  void angularMomentum(const Eigen::Vector3d& amom) { amom_ = amom; }

      // robot joint variables
	  RobotPosture& robotPosture() { return robot_posture_; }
	  RobotVelocity& robotVelocity() { return robot_velocity_; }
	  RobotAcceleration& robotAcceleration() { return robot_acceleration_; }

	  const RobotPosture& robotPosture() const { return robot_posture_; }
	  const RobotVelocity& robotVelocity() const { return robot_velocity_; }
	  const RobotAcceleration& robotAcceleration() const { return robot_acceleration_; }

	  void robotPosture(const RobotPosture& robot_posture) { robot_posture_ = robot_posture; }
	  void robotVelocity(const RobotVelocity& robot_velocity) { robot_velocity_ = robot_velocity; }
	  void robotAcceleration(const RobotAcceleration& robot_acceleration) { robot_acceleration_ = robot_acceleration; }

      // endeffector poses
      Eigen::Vector3d& endeffectorPosition(int eff_id) { return eff_positions_[eff_id]; }
      Eigen::Vector3d& endeffectorVelocity(int eff_id) { return eff_velocities_[eff_id]; }
      Eigen::Vector3d& endeffectorAcceleration(int eff_id) { return eff_accelerations_[eff_id]; }
      Eigen::Quaternion<double>& endeffectorOrientation(int eff_id) { return eff_orientations_[eff_id]; }

      const Eigen::Vector3d& endeffectorPosition(int eff_id) const { return eff_positions_[eff_id]; }
      const Eigen::Vector3d& endeffectorVelocity(int eff_id) const { return eff_velocities_[eff_id]; }
      const Eigen::Vector3d& endeffectorAcceleration(int eff_id) const { return eff_accelerations_[eff_id]; }
      const Eigen::Quaternion<double>& endeffectorOrientation(int eff_id) const { return eff_orientations_[eff_id]; }

      const std::vector<Eigen::Vector3d>& pyEndeffectorPositions() const { return eff_positions_; }
      void pyEndeffectorPositions(const std::vector<Eigen::Vector3d>& eff_positions) { eff_positions_ = eff_positions; }

	  // Helper functions
	  std::string toString() const;
	  const int& numJoints() const { return num_joints_; }
	  friend std::ostream& operator<<(std::ostream &os, const KinematicsState& obj) { return os << obj.toString(); }

    private:
	  RobotPosture robot_posture_;
	  RobotVelocity robot_velocity_;
	  RobotAcceleration robot_acceleration_;

      int num_joints_;
      Eigen::Vector3d com_, lmom_, amom_;
      std::vector<Eigen::Quaternion<double>> eff_orientations_;
      std::vector<Eigen::Vector3d> eff_positions_, eff_velocities_, eff_accelerations_;
  };

  /**
   * This class is a container for a sequence of kinematic states
   */
  class KinematicsSequence
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
	  KinematicsSequence(){}
	  ~KinematicsSequence(){}

	  void clean() { kinematics_sequence_.clear(); }
	  void resize(int num_timesteps, int num_joints);
	  int size() const { return kinematics_sequence_.size(); }

	  const std::vector<KinematicsState>& kinematicsSequence() const { return kinematics_sequence_; }
	  void kinematicsSequence(const std::vector<KinematicsState>& kinematics_sequence) { kinematics_sequence_ = kinematics_sequence; }

	  KinematicsState& kinematicsState(int time_id) { return kinematics_sequence_[time_id]; }
	  const KinematicsState& kinematicsState(int time_id) const { return kinematics_sequence_[time_id]; }

	  std::string toString() const;
  	  friend std::ostream& operator<<(std::ostream &os, const KinematicsSequence& obj) { return os << obj.toString(); }

    private:
      std::vector<KinematicsState> kinematics_sequence_;
  };

}
