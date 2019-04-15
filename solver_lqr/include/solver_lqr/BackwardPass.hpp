#pragma once

#include <solver_lqr/ForwardPass.hpp>
#include <solver/interface/Solver.hpp>
#include <solver_lqr/FiniteDifferences.hpp>

namespace solverlqr {

  class BackwardPass
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
	  BackwardPass() : is_initialized_(false) {}
      virtual ~BackwardPass(){}

      void initialize(OcpBase* ocp, const SolverLqrSetting& stgs);
      void optimizeController(const ForwardPass& fpass, const FiniteDifferences& devs, double lambda);

      const Eigen::Vector2d& dV() const { return dV_; }
      ControlSequence& controlSeq() { return control_seq_; }
      const bool& hasDiverged() const { return diverge_flag_; }
      const int& divergeIteration() const { return diverge_iteration_; }
      const ControlSequence& controlSeq() const { return control_seq_; }

    private:

      OcpBase& getOcp() { return *ocp_; }
      const OcpBase& getOcp() const { return *ocp_; }
      const SolverLqrSetting& getLqrSetting() const { return *stgs_; }
      Eigen::MatrixXd tensorContraction(const Eigen::VectorXd& vz, const std::vector<Eigen::MatrixXd>& fzz);
      void findConstrainedControls(int time_id, const Eigen::MatrixXd& chol_hessian, const Eigen::VectorXd& gradient);

    private:
      Eigen::VectorXd V_;
      Eigen::Vector2d dV_;
      Eigen::VectorXd Qu_;                               // nu * 1
      Eigen::VectorXd Qx_;                               // nx * 1
      Eigen::MatrixXd Qxx_, VxxR_;                       // nx * nx
      Eigen::MatrixXd Qxu_, QxuR_;                       // nx * nu
      Eigen::MatrixXd Quu_, QuuR_;                       // nu * nu
      std::vector<Eigen::VectorXd> Vx_, Vxh_;            // nx * 1  * horizon+1
      std::vector<Eigen::MatrixXd> Vxx_, Vxxh_, Vxhxh_;  // nx * nx * horizon+1

      solver::Model model_;
      solver::LinExpr lin_expr_;
      std::vector<solver::Var> vars_;
      solver::DCPQuadExpr quad_expr_;

      OcpBase* ocp_;
      ControlSequence control_seq_;
      const SolverLqrSetting* stgs_;
      Eigen::LLT<Eigen::MatrixXd> llt_;
      bool diverge_flag_, is_initialized_;
      int diverge_iteration_, tdim_, xdim_, udim_;
  };

}
