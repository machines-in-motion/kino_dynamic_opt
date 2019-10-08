/**
 * @file ContactPlanInterface.hpp
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
#include <momentumopt/cntopt/ContactState.hpp>
#include <momentumopt/dynopt/DynamicsState.hpp>
#include <momentumopt/setting/PlannerSetting.hpp>
#include <momentumopt/cntopt/TerrainDescription.hpp>
#include <momentumopt/utilities/TrajectoryGenerator.hpp>

namespace momentumopt {

  class KinematicsOptimizer;

  /**
   * This class implements a simple interface to store a sequence of
   * contacts, including its activation and de-activation time, position
   * and orientation of the contact, and a variable (set to 1) indicating
   * if the forces at this contact are subject to friction cone constraints
   * or (set to 2) if not, such as in the case of hands grasping bars.
   * It also fills the corresponding fields of DynamicsState within the
   * DynamicsSequence, namely contact positions, orientations, activations,
   * to be used by the dynamics optimizer.
   */
  class ContactPlanInterface
  {
    public:
      ContactPlanInterface(){}
      ~ContactPlanInterface() {
          // printf("planner_setting_=%p\n", (void*)planner_setting_);
      }

      void initialize(const PlannerSetting& planner_setting);
      virtual void optimize(const DynamicsState& ini_state, const TerrainDescription& terrain) = 0;
      void fillDynamicsSequence(const DynamicsState& ini_state, DynamicsSequence& dynamics_sequence);
      void updateEndeffectorTrajectories(const DynamicsState& ini_state, DynamicsSequence& dynamics_sequence);

      ContactSequence& contactSequence() { return contact_sequence_; }
      const ContactSequence& contactSequence() const { return contact_sequence_; }

      ViapointSequence& viapointSequence() { return viapoint_sequence_; }
      const ViapointSequence& viapointSequence() const { return viapoint_sequence_; }

    private:
      double mapIdToContactTime(const DynamicsSequence& dyn_sequence, int map_id);
      int findMakeContactId(const DynamicsSequence& dyn_seq, const int search_start_id, int eff_id);
      int findBreakContactId(const DynamicsSequence& dyn_seq, const int search_start_id, int eff_id);
      void findActiveViapoints(Eigen::Vector3d& viapoint_position, int eff_id, int cnt_id);
      void generateFlightPhase(DynamicsSequence& dyn_seq, int start_id, int end_id, int eff_id,
                               const Eigen::Quaternion<double>& qini, const Eigen::Quaternion<double>& qend);

    protected:
      /*! Getter and setter methods for getting the planner variables  */
      inline const PlannerSetting& getSetting() const { return *planner_setting_; }

    protected:
      TrajectoryGenerator sw_traj_;
      ContactSequence contact_sequence_;
      ViapointSequence viapoint_sequence_;
      const PlannerSetting* planner_setting_;
  };
}
