#pragma once

#include <cmath>
#include <vector>
#include <solver_lqr/OcpDescription.hpp>

namespace solverlqr {

  class FiniteDifferences
  {
    public:
      FiniteDifferences() : h_devs_(std::pow(2.,-17)), is_initialized_(false) {}
      ~FiniteDifferences(){}

      void initialize(OcpBase* ocp_description,
          bool compute_objective_first_derivatives = false, bool compute_dynamics_first_derivatives = false,
          bool compute_objective_second_derivatives = false, bool compute_dynamics_second_derivatives = false);
      const OcpBase* getOcpDescription() const { return ocp_; }
      void computeDerivatives();

      Eigen::MatrixXd& fx(int time_id) { return fx_seq_[time_id]; }
      Eigen::MatrixXd& fu(int time_id) { return fu_seq_[time_id]; }
      const Eigen::MatrixXd& fx(int time_id) const { return fx_seq_[time_id]; }
      const Eigen::MatrixXd& fu(int time_id) const { return fu_seq_[time_id]; }
      std::vector<Eigen::MatrixXd>& fxx(int time_id) { return fxx_seq_[time_id]; }
      std::vector<Eigen::MatrixXd>& fxu(int time_id) { return fxu_seq_[time_id]; }
      std::vector<Eigen::MatrixXd>& fuu(int time_id) { return fuu_seq_[time_id]; }
      const std::vector<Eigen::MatrixXd>& fxx(int time_id) const { return fxx_seq_[time_id]; }
      const std::vector<Eigen::MatrixXd>& fxu(int time_id) const { return fxu_seq_[time_id]; }
      const std::vector<Eigen::MatrixXd>& fuu(int time_id) const { return fuu_seq_[time_id]; }

      Eigen::VectorXd& cx(int time_id) { return cx_seq_[time_id]; }
      Eigen::VectorXd& cu(int time_id) { return cu_seq_[time_id]; }
      Eigen::MatrixXd& cxx(int time_id) { return cxx_seq_[time_id]; }
      Eigen::MatrixXd& cxu(int time_id) { return cxu_seq_[time_id]; }
      Eigen::MatrixXd& cuu(int time_id) { return cuu_seq_[time_id]; }
      const Eigen::VectorXd& cx(int time_id) const { return cx_seq_[time_id]; }
      const Eigen::VectorXd& cu(int time_id) const { return cu_seq_[time_id]; }
      const Eigen::MatrixXd& cxx(int time_id) const { return cxx_seq_[time_id]; }
      const Eigen::MatrixXd& cxu(int time_id) const { return cxu_seq_[time_id]; }
      const Eigen::MatrixXd& cuu(int time_id) const { return cuu_seq_[time_id]; }

    private:
      void computeDynamicsFirstDerivatives();
      void computeDynamicsSecondDerivatives();
      void computeObjectiveFirstDerivatives();
      void computeObjectiveSecondDerivatives();

    private:
      std::vector<Eigen::VectorXd> cx_seq_;                // nx * 1 * horizon+1
      std::vector<Eigen::VectorXd> cu_seq_;                // nu * 1 * horizon+1
      std::vector<Eigen::MatrixXd> cxx_seq_;               // nx * nx * horizon+1
      std::vector<Eigen::MatrixXd> cxu_seq_;               // nx * nu * horizon+1
      std::vector<Eigen::MatrixXd> cuu_seq_;               // nu * nu * horizon+1

      std::vector<Eigen::MatrixXd> fx_seq_;                // nx * nx * horizon
      std::vector<Eigen::MatrixXd> fu_seq_;                // nx * nu * horizon
      std::vector<std::vector<Eigen::MatrixXd>> fxx_seq_;  // nx * nx * nx * horizon
      std::vector<std::vector<Eigen::MatrixXd>> fxu_seq_;  // nx * nx * nu * horizon
      std::vector<std::vector<Eigen::MatrixXd>> fuu_seq_;  // nx * nu * nu * horizon

      OcpBase* ocp_;
      double h_devs_;
      int tdim_, xdim_, udim_;
      bool is_initialized_, compute_objective_first_derivatives_, compute_dynamics_first_derivatives_,
	       compute_objective_second_derivatives_, compute_dynamics_second_derivatives_;
  };

}
