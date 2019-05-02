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
          printf("planner_setting_=%p\n", planner_setting_);
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
