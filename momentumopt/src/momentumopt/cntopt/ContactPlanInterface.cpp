/**
 * @file ContactPlanInterface.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#include <momentumopt/cntopt/ContactPlanInterface.hpp>

namespace momentumopt {

  // ContactPlanInterface
  void ContactPlanInterface::initialize(const PlannerSetting& planner_setting)
  {
    planner_setting_ = &planner_setting;
  }

  void ContactPlanInterface::fillDynamicsSequence(const DynamicsState& /*ini_state*/, DynamicsSequence& dynamics_sequence)
  {
	// configuration of end-effectors during contact
    dynamics_sequence.activeEndeffectorSteps().setZero();
    for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
      int counter = 0;
      int ini_id = 0, end_id = contact_sequence_.endeffectorContacts(eff_id).size();
      for (int cnt_id=ini_id; cnt_id<end_id; cnt_id++) {
        if (contact_sequence_.endeffectorContacts(eff_id)[cnt_id].selectedAsActive()) {
          for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
            double current_time = double(time_id+1.0)*this->getSetting().get(PlannerDoubleParam_TimeStep);
            if (current_time>=contact_sequence_.endeffectorContacts(eff_id)[cnt_id].contactActivationTime() &&
                current_time< contact_sequence_.endeffectorContacts(eff_id)[cnt_id].contactDeactivationTime())
            {
              dynamics_sequence.dynamicsState(time_id).endeffectorActivation(eff_id) = true;
              dynamics_sequence.dynamicsState(time_id).endeffectorContactId(eff_id) = cnt_id;
              dynamics_sequence.dynamicsState(time_id).endeffectorActivationId(eff_id) = counter++;
              dynamics_sequence.dynamicsState(time_id).endeffectorPosition(eff_id) = contact_sequence_.endeffectorContacts(eff_id)[cnt_id].contactPosition();
              dynamics_sequence.dynamicsState(time_id).endeffectorOrientation(eff_id) = contact_sequence_.endeffectorContacts(eff_id)[cnt_id].contactOrientation();
            }
          }
        }
      }
      dynamics_sequence.activeEndeffectorSteps()[eff_id] = counter;
    }
  }

  // helper function to build end-effector flight phases
  void ContactPlanInterface::updateEndeffectorTrajectories(const DynamicsState& ini_state, DynamicsSequence& dynamics_sequence)
  {
    if (this->getSetting().get(PlannerBoolParam_LoadKinematics))
    {
      Eigen::Vector3d viapoint_position;
      for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
        if (this->contactSequence().numEndeffectorContacts(eff_id)>0) {
          int ini_id = 0, end_id = 0;

          // configuration of end-effectors first flight phase, when not in contact
          if (!ini_state.endeffectorActivation(eff_id)) {
            ini_id = 0;
            end_id = findMakeContactId(dynamics_sequence, ini_id, eff_id);
            this->findActiveViapoints(viapoint_position, eff_id, -1);
            sw_traj_.initialize(this->mapIdToContactTime(dynamics_sequence, ini_id+1), ini_state.endeffectorPosition(eff_id),
                                this->mapIdToContactTime(dynamics_sequence, end_id+1), contact_sequence_.endeffectorContacts(eff_id)[0].contactPosition(),
                                viapoint_position);
            this->generateFlightPhase(dynamics_sequence, ini_id, end_id, eff_id, ini_state.endeffectorOrientation(eff_id), contact_sequence_.endeffectorContacts(eff_id)[0].contactOrientation());
          }

          // configuration of end-effectors during flight phases
          for (int cnt_id=0; cnt_id<contact_sequence_.numEndeffectorContacts(eff_id)-1; cnt_id++) {
            ini_id = findBreakContactId(dynamics_sequence, end_id, eff_id);
            end_id = findMakeContactId(dynamics_sequence, ini_id, eff_id);
            this->findActiveViapoints(viapoint_position, eff_id, cnt_id);
            sw_traj_.initialize(this->mapIdToContactTime(dynamics_sequence, ini_id+1), contact_sequence_.endeffectorContacts(eff_id)[cnt_id].contactPosition(),
                                this->mapIdToContactTime(dynamics_sequence, end_id+1), contact_sequence_.endeffectorContacts(eff_id)[cnt_id+1].contactPosition(),
                                viapoint_position);
            this->generateFlightPhase(dynamics_sequence, ini_id, end_id, eff_id, contact_sequence_.endeffectorContacts(eff_id)[cnt_id].contactOrientation(), contact_sequence_.endeffectorContacts(eff_id)[cnt_id+1].contactOrientation());
          }

          // configuration of end-effectors last flight phase, when not in contact
          if (!dynamics_sequence.dynamicsState(this->getSetting().get(PlannerIntParam_NumTimesteps)-1).endeffectorActivation(eff_id)) {
            ini_id = findBreakContactId(dynamics_sequence, end_id, eff_id);
            end_id = this->getSetting().get(PlannerIntParam_NumTimesteps)-1;
            this->findActiveViapoints(viapoint_position, eff_id, contact_sequence_.numEndeffectorContacts(eff_id)-1);
            sw_traj_.initialize(this->mapIdToContactTime(dynamics_sequence, ini_id+1), contact_sequence_.endeffectorContacts(eff_id).back().contactPosition(),
                                this->mapIdToContactTime(dynamics_sequence, end_id+1), ini_state.endeffectorPosition(eff_id)+this->getSetting().get(PlannerVectorParam_CenterOfMassMotion),
                                viapoint_position);
            this->generateFlightPhase(dynamics_sequence, ini_id, end_id, eff_id, contact_sequence_.endeffectorContacts(eff_id).back().contactOrientation(), ini_state.endeffectorOrientation(eff_id));
          }
        }
      }
    }
  }

  double ContactPlanInterface::mapIdToContactTime(const DynamicsSequence& dyn_sequence, int map_id)
  {
    double mapped_time = 0.0;
    for (int time_id=0; time_id<map_id; time_id++)
      mapped_time += dyn_sequence.dynamicsState(time_id).time();
    return mapped_time;
  }

  int ContactPlanInterface::findMakeContactId(const DynamicsSequence& dyn_sequence, const int search_start_id, int eff_id)
  {
	int make_cnt_id = 0, search_id = search_start_id;
  	while (true) {
  	  if (dyn_sequence.dynamicsState(search_id).endeffectorActivation(eff_id)==false &&
  		  dyn_sequence.dynamicsState(search_id+1).endeffectorActivation(eff_id)==true )
  	  {
  		make_cnt_id = search_id;
  		break;
  	  }
      search_id++;
  	}
  	return make_cnt_id;
  }

  int ContactPlanInterface::findBreakContactId(const DynamicsSequence& dyn_sequence, const int search_start_id, int eff_id)
  {
	int break_cnt_id = 0, start_id = search_start_id;
  	while (true) {
  	  if (dyn_sequence.dynamicsState(start_id  ).endeffectorActivation(eff_id)==true  &&
  		  dyn_sequence.dynamicsState(start_id+1).endeffectorActivation(eff_id)==false )
  	  {
  		break_cnt_id = start_id+1;
  		break;
  	  }
  	start_id++;
  	}
  	return break_cnt_id;
  }

  void ContactPlanInterface::findActiveViapoints(Eigen::Vector3d& viapoint_position, int eff_id, int cnt_id)
  {
    for (int via_id=0; via_id<this->viapointSequence().numEndeffectorViapoints(eff_id); via_id++)
      if (this->viapointSequence().endeffectorViapoints(eff_id)[via_id].viapointId()==cnt_id) {
        viapoint_position = viapoint_sequence_.endeffectorViapoints(eff_id)[via_id].viapointPosition();
        break;
      }
  }

  void ContactPlanInterface::generateFlightPhase(DynamicsSequence& dynamics_sequence, int start_id, int end_id, int eff_id,
		                           const Eigen::Quaternion<double>& qini, const Eigen::Quaternion<double>& qend)
  {
    for (int time_id=start_id; time_id<=end_id; time_id++) {
      double time = this->mapIdToContactTime(dynamics_sequence, time_id+1);
      sw_traj_.update(time);
      dynamics_sequence.dynamicsState(time_id).endeffectorPosition(eff_id) = sw_traj_.desPos();
      dynamics_sequence.dynamicsState(time_id).endeffectorVelocity(eff_id) = sw_traj_.desVel();
      dynamics_sequence.dynamicsState(time_id).endeffectorAcceleration(eff_id) = sw_traj_.desAcc();

      double query_time = std::min(2.0*double(time_id-start_id)/double(end_id+1.0-start_id), 1.0);
      dynamics_sequence.dynamicsState(time_id).endeffectorOrientation(eff_id) = qini.slerp(query_time, qend);
    }
  }


}
