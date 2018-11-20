#pragma once

#include <solver_lqr/SolverLqr.hpp>
#include <solver_lqr/OcpDescription.hpp>
#include <momentumopt/dynopt/DynamicsState.hpp>
#include <momentumopt/setting/PlannerSetting.hpp>

namespace momentumopt {

  /** Class that implements the dynamic description of momentum dynamics as an optimal
   *  control problem, to be used by an LQR algorithm to derive a feedback sequence.
   */
  class DynamicsFeedback : public solverlqr::OcpBase
  {
    public:
	  /*! default class constructor and destructor */
	  DynamicsFeedback() {}
      ~DynamicsFeedback() {}

      /*! stores a pointer to the planner setting and reference dynamics sequence */
      void setPlannerSetting(const PlannerSetting& setting) { setting_ = &setting; }
      void setStatesAndControls(const DynamicsState& ini_state, const DynamicsSequence& dyn_sequence);

    private:
      /*! function to give the user the possibility to read custom parameters */
      void configure(const YAML::Node& user_parameters);

      /*! definition of the objective function for the optimal control problem */
      double objective(const solverlqr::StateBase& state, const solverlqr::ControlBase& control, int time_id, bool is_final_timestep);

      /*! definition of the dynamics evolution for the optimal control problem */
      solverlqr::StateBase dynamics(const solverlqr::StateBase& state, const solverlqr::ControlBase& control, int time_id);

      /*! these functions give access to the planner setting and optimized motion reference plan */
      inline const PlannerSetting& getSetting() const { return *setting_; }
      inline const DynamicsSequence& dynamicsSequence() const { return *ref_dyn_sequence_; }

    private:
      /*! pointer to planner setting */
      const PlannerSetting* setting_;

      /*! pointer to dynamic sequence to be tracked */
      const DynamicsSequence* ref_dyn_sequence_;

      /*! helper vector variables for the optimization problem */
      Eigen::MatrixXd control_cost_, com_tracking_, lmom_tracking_, amom_tracking_,
                      com_final_tracking_, lmom_final_tracking_, amom_final_tracking_;
  };

  /*
   * Wrapper class for easily using DynamicsFeedback within Python
   */
  class DynamicsFeedbackWrapper
  {
    public:
      DynamicsFeedbackWrapper(){}
      ~DynamicsFeedbackWrapper(){}

      void initialize(solverlqr::SolverLqrSetting& lqr_stg, const PlannerSetting& plan_stg);
      void optimize(const DynamicsState& ini_state, const DynamicsSequence& dynamics_sequence);
      Eigen::MatrixXd forceGain(int time_id);

    private:
      const PlannerSetting* plan_stg_;
      solverlqr::SolverLqrSetting* lqr_stg_;

      DynamicsFeedback dynamics_description_;
      solverlqr::SolverLqr dynamics_lqrsolver_;
  };
}
