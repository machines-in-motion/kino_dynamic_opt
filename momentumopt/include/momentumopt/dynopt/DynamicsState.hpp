/**
 * @file DynamicsState.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#pragma once

#include <array>
#include <vector>
#include <string>
#include <iostream>
#include <Eigen/Eigen>

#include <momentumopt/setting/Definitions.hpp>
#include <momentumopt/cntopt/ContactState.hpp>

namespace momentumopt {

  /**
   * This class is a container for all variables required to define a
   * dynamic state: center of mass position, linear and angular momenta
   * and its rates; forces, torques and center of pressure of the end-
   * effectors; positions, orientations, activations and contact types
   * of each end-effector.
   */
  struct DynamicsState
  {
    public:
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
	  DynamicsState();
	  ~DynamicsState(){}

	  // Center of mass, linear and angular momenta
	  double& time() { return dtime_; }
	  Eigen::Vector3d& centerOfMass() { return com_; }
	  Eigen::Vector3d& linearMomentum() { return lmom_; }
	  Eigen::Vector3d& angularMomentum() { return amom_; }
	  Eigen::Vector3d& linearMomentumRate() { return lmomd_; }
	  Eigen::Vector3d& angularMomentumRate() { return amomd_; }

	  const double& time() const { return dtime_; }
	  const Eigen::Vector3d& centerOfMass() const { return com_; }
	  const Eigen::Vector3d& linearMomentum() const { return lmom_; }
	  const Eigen::Vector3d& angularMomentum() const { return amom_; }
	  const Eigen::Vector3d& linearMomentumRate() const { return lmomd_; }
	  const Eigen::Vector3d& angularMomentumRate() const { return amomd_; }

	  void time(const double& dtime) { dtime_ = dtime; }
	  void centerOfMass(const Eigen::Vector3d& com) { com_ = com; }
	  void linearMomentum(const Eigen::Vector3d& lmom) { lmom_ = lmom; }
	  void angularMomentum(const Eigen::Vector3d& amom) { amom_ = amom; }
	  void linearMomentumRate(const Eigen::Vector3d& lmomd) { lmomd_ = lmomd; }
	  void angularMomentumRate(const Eigen::Vector3d& amomd) { amomd_ = amomd; }

      // Endeffector forces, torques and cops
      Eigen::Vector3d& endeffectorCoP(int eff_id) { return eff_cops_[eff_id]; }
      Eigen::Vector3d& endeffectorForce(int eff_id) { return eff_forces_[eff_id]; }
      Eigen::Vector3d& endeffectorTorque(int eff_id) { return eff_torques_[eff_id]; }
      Eigen::Vector3d& endeffectorTorqueAtContactPoint(int eff_id) { return eefs_trqs_contact_point_[eff_id]; }

      long endeffectorNum() { return eff_cops_.size(); }
      const Eigen::Vector3d& endeffectorCoP(int eff_id) const { return eff_cops_[eff_id]; }
      const Eigen::Vector3d& endeffectorForce(int eff_id) const { return eff_forces_[eff_id]; }
      const Eigen::Vector3d& endeffectorTorque(int eff_id) const { return eff_torques_[eff_id]; }
      const Eigen::Vector3d& endeffectorTorqueAtContactPoint(int eff_id) const { return eefs_trqs_contact_point_[eff_id]; }

      const std::vector<Eigen::Vector3d>& pyEndeffectorCops() const { return eff_cops_; }
      const std::vector<Eigen::Vector3d>& pyEndeffectorForces() const { return eff_forces_; }
      const std::vector<Eigen::Vector3d>& pyEndeffectorTorques() const { return eff_torques_; }

      void pyEndeffectorCops(const std::vector<Eigen::Vector3d>& eff_cops) { eff_cops_ = eff_cops; }
      void pyEndeffectorForces(const std::vector<Eigen::Vector3d>& eff_forces) { eff_forces_ = eff_forces; }
      void pyEndeffectorTorques(const std::vector<Eigen::Vector3d>& eff_torques) { eff_torques_ = eff_torques; }

	  // Endeffector activations, activation and contact ids
      int& endeffectorContactId(int eff_id) { return cnt_ids_[eff_id]; }
      int& endeffectorActivationId(int eff_id) { return eff_ids_[eff_id]; }
      bool& endeffectorActivation(int eff_id) { return eff_activations_[eff_id]; }

      const int& endeffectorContactId(int eff_id) const { return cnt_ids_[eff_id]; }
      const int& endeffectorActivationId(int eff_id) const { return eff_ids_[eff_id]; }
      bool endeffectorActivation(int eff_id) const { return eff_activations_[eff_id]; }

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
	  friend std::ostream& operator<<(std::ostream &os, const DynamicsState& obj) { return os << obj.toString(); }
	  void fillInitialRobotState(const std::string cfg_file, const std::string robot_state = "initial_robot_configuration");

    private:
	  double dtime_;
	  Eigen::Vector3d com_, amom_, lmom_, amomd_, lmomd_;
      std::array<bool, Problem::n_endeffs_> eff_activations_;
      std::array<int, Problem::n_endeffs_> eff_ids_, cnt_ids_;

      std::vector<Eigen::Quaternion<double>> eff_orientations_;
      std::vector<Eigen::Vector3d> eff_positions_, eff_velocities_, eff_accelerations_;
      std::vector<Eigen::Vector3d> eff_forces_, eff_torques_, eff_cops_, eefs_trqs_contact_point_;
  };

  /**
   * This class is a container for a sequence of dynamic states,
   * for all time steps in the optimization.
   */
  class DynamicsSequence
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
	  DynamicsSequence(){}
	  ~DynamicsSequence(){}

	  void resize(int num_timesteps);
	  void clean() { dynamics_sequence_.clear(); }
	  int size() const { return dynamics_sequence_.size(); }

	  const std::vector<DynamicsState>& dynamicsSequence() const { return dynamics_sequence_; }
	  void dynamicsSequence(const std::vector<DynamicsState>& dynamics_sequence) { dynamics_sequence_ = dynamics_sequence; }

      DynamicsState& dynamicsState(int time_id) { return dynamics_sequence_[time_id]; }
	  const DynamicsState& dynamicsState(int time_id) const { return dynamics_sequence_[time_id]; }

	  Eigen::Matrix<int, Problem::n_endeffs_, 1>& activeEndeffectorSteps() { return active_endeffector_steps_; }
	  const Eigen::Matrix<int, Problem::n_endeffs_, 1>& activeEndeffectorSteps() const { return active_endeffector_steps_; }

	  std::string toString() const;
  	  friend std::ostream& operator<<(std::ostream &os, const DynamicsSequence& obj) { return os << obj.toString(); }

    private:
      std::vector<DynamicsState> dynamics_sequence_;
      Eigen::Matrix<int, Problem::n_endeffs_, 1> active_endeffector_steps_;
  };

}
