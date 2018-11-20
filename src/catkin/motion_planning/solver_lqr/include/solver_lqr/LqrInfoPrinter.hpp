#pragma once

#include <string>
#include <vector>
#include <memory>
#include <solver_lqr/SolverLqrSetting.hpp>

namespace solverlqr {

  enum class Color {red, blue, green, yellow, magenta, cyan, white, bold_red, bold_blue,
	                bold_green, bold_yellow, bold_magenta, bold_cyan, bold_white};

  enum class MsgLqr {
    STAT_cholesky,      /*! Status msg: Cholesky decomposition failed       */
    STAT_success,       /*! Status msg: LqrOptimization was a success       */
    STAT_rejection,     /*! Status msg: Iteration or update rejected        */
    STAT_begin,         /*! Status msg: Initial print message               */
    STAT_summary,       /*! Status msg: Summary at optimization end         */
    EXIT_divergence,    /*! Exit msg: Termination due to divergence         */
    EXIT_maxlambda,     /*! Exit msg: Termination due to max regularization */
    EXIT_maxiter,       /*! Exit msg: Termination due to max iterations     */
    SUCC_costchange,    /*! Success msg: Minimum tolerance in cost change   */
    SUCC_gradient,      /*! Success msg: Minimum tolerance in contro change */
  };

  /**
   * Helper class that contains information about the status of the optimization problem
   */
  class LqrOptimizationInfo
  {
    public:
	  LqrOptimizationInfo(){}
	  ~LqrOptimizationInfo(){}

      int& get(const SolverLqrIntParam& param);
      double& get(const SolverLqrDoubleParam& param);

      const int& get(const SolverLqrIntParam& param) const;
      const double& get(const SolverLqrDoubleParam& param) const;

    private:
      int current_iteration_, backpass_diverge_iteration_;
      double cost_, cost_change_, expected_cost_, control_gradient_, current_regularization_;
  };

  class LqrInfoPrinter
  {
    public:
      LqrInfoPrinter();
      virtual ~LqrInfoPrinter(){}

      void initialize(const SolverLqrSetting& stgs);
      void display(const MsgLqr& msg, const LqrOptimizationInfo& info);

    private:
      void enter();
      std::string color_picker(const Color& color);
      void printMsg(const std::string& str, const Color& color);
      void split(const std::string& s, char delim, std::vector<std::string>& v_str, std::vector<std::string>& v_prec);
      void printMsg(const std::string& str, std::vector<double>& v_val, std::vector<bool>& v_prec, const Color& text_color = Color::white, const Color& num_color = Color::white );

      inline const SolverLqrSetting& getLqrSetting() const { return *stgs_; }

    private:
      std::shared_ptr<SolverLqrSetting> stgs_;
      std::string close_, open_red_, open_blue_, open_cyan_, open_green_, open_white_, open_yellow_,
                  open_magenta_, open_bold_red_, open_bold_blue_, open_bold_cyan_, open_bold_green_,
                  open_bold_white_, open_bold_yellow_, open_bold_magenta_, prec_;
  };
}
