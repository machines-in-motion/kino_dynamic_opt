#pragma once

#include <solver_lqr/OcpDescription.hpp>
#include <solver_lqr/SolverLqrSetting.hpp>

namespace solverlqr {

  class ForwardPass
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
      ForwardPass(){}
      virtual ~ForwardPass(){}

      void initialize(OcpBase* ocp, const SolverLqrSetting& stgs);
      void applyController(const StateSequence& nom_state_seq, const ControlSequence& nom_control_seq);

      const double& newCost() const { return new_cost_; }
      const bool& hasDiverged() const { return has_diverged_; }
      const double& timestepCost(int time_id) const { return timestep_cost_[time_id]; }

      StateSequence& stateSeq() { return new_state_seq_; }
      ControlSequence& controlSeq() { return new_control_seq_; }
      const StateSequence& stateSeq() const { return new_state_seq_; }
      const ControlSequence& controlSeq() const { return new_control_seq_; }

    private:
      OcpBase& getOcp() { return *ocp_; }
      const OcpBase& getOcp() const { return *ocp_; }
      const SolverLqrSetting& getLqrSetting() const { return *stgs_; }

    private:
      OcpBase* ocp_;
      const SolverLqrSetting* stgs_;

      StateBase dx_;
      StateSequence new_state_seq_;
      ControlSequence new_control_seq_;

      double new_cost_;
      bool has_diverged_;
      Eigen::VectorXd timestep_cost_;
  };

}
