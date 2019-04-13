#pragma once

#include <solver_lqr/BackwardPass.hpp>
#include <solver_lqr/LqrInfoPrinter.hpp>

namespace solverlqr {

  class SolverLqr
  {
    public:
      SolverLqr();
      virtual ~SolverLqr(){}

      void initialize(OcpBase* ocp, SolverLqrSetting& setting);
      void optimize();

      FiniteDifferences& getFiniteDiff() { return devs_; }
      const FiniteDifferences& getFiniteDiff() const { return devs_; }

      void loadSolution(const std::string& load_file);

    private:
      void storeSolution();

      OcpBase& getOcp() { return *ocp_; }
      ForwardPass& getForwPass() { return fpass_; }
      BackwardPass& getBackPass() { return bpass_; }
      LqrInfoPrinter& getPrinter() { return printer_; }
      SolverLqrSetting& getLqrSetting() { return *setting_; }
      LqrOptimizationInfo& getLqrInfo() { return lqr_optimization_info_; }

      const OcpBase& getOcp() const { return *ocp_; }
      const ForwardPass& getForwPass() const { return fpass_; }
      const BackwardPass& getBackPass() const { return bpass_; }
      const LqrInfoPrinter& getPrinter() const { return printer_; }
      const SolverLqrSetting& getLqrSetting() const { return *setting_; }
      const LqrOptimizationInfo& getLqrInfo() const { return lqr_optimization_info_; }

    private:
      OcpBase* ocp_;
      ForwardPass fpass_;
      BackwardPass bpass_;
      FiniteDifferences devs_;
      LqrInfoPrinter printer_;
      SolverLqrSetting* setting_;
      LqrOptimizationInfo lqr_optimization_info_;

      bool converged_, devs_flag_, backpass_flag_, forwpass_flag_;
      double linesearch_coeff_, mult_regularization_change_, expected_improvement_;
  };

}
