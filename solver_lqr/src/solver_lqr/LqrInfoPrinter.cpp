#include <cmath>
#include <sstream>
#include <iostream>
#include <yaml_cpp_catkin/yaml_eigen.h>
#include <solver_lqr/LqrInfoPrinter.hpp>

namespace solverlqr {

  // LqrOptimizationInfo class functions implementations
  int& LqrOptimizationInfo::get(const SolverLqrIntParam& param)
  {
    switch(param)
    {
      case SolverLqrIntParam_CurrentIteration: { return current_iteration_; }
      case SolverLqrIntParam_BackpassDivergeIteration: { return backpass_diverge_iteration_; }
      default: { throw std::runtime_error("LqrOptimizationInfo::get SolverLqrIntParam invalid"); break; }
    }
  }

  double& LqrOptimizationInfo::get(const SolverLqrDoubleParam& param)
  {
    switch(param)
    {
      case SolverLqrDoubleParam_Cost: { return cost_; }
      case SolverLqrDoubleParam_CostChange: { return cost_change_; }
      case SolverLqrDoubleParam_ExpectedCost : { return expected_cost_; }
      case SolverLqrDoubleParam_ControlGradient: { return control_gradient_; }
      case SolverLqrDoubleParam_CurrentRegularization: { return current_regularization_; }
      default: { throw std::runtime_error("LqrOptimizationInfo::get SolverLqrDoubleParam invalid"); break; }
    }
  }

  const int& LqrOptimizationInfo::get(const SolverLqrIntParam& param) const
  {
    switch(param)
    {
      case SolverLqrIntParam_CurrentIteration: { return current_iteration_; }
      case SolverLqrIntParam_BackpassDivergeIteration: { return backpass_diverge_iteration_; }
      default: { throw std::runtime_error("LqrOptimizationInfo::get SolverLqrIntParam invalid"); break; }
    }
  }

  const double& LqrOptimizationInfo::get(const SolverLqrDoubleParam& param) const
  {
    switch(param)
    {
      case SolverLqrDoubleParam_Cost: { return cost_; }
      case SolverLqrDoubleParam_CostChange: { return cost_change_; }
      case SolverLqrDoubleParam_ExpectedCost : { return expected_cost_; }
      case SolverLqrDoubleParam_ControlGradient: { return control_gradient_; }
      case SolverLqrDoubleParam_CurrentRegularization: { return current_regularization_; }
      default: { throw std::runtime_error("LqrOptimizationInfo::get SolverLqrDoubleParam invalid"); break; }
    }
  }

  std::string toString(double number, int precision, bool is_scientific=true)
  {
    std::stringstream ss;
    if (is_scientific) { ss << std::scientific << std::setprecision(precision) << number; }
    else               { ss << std::fixed << std::setprecision(precision) << number; }
    return ss.str();
  }

  // LqrInfoPrinter class helper functions implementations
  LqrInfoPrinter::LqrInfoPrinter()
    : close_("\033[0m"),
      open_red_("\033[0;31m"),
      open_blue_("\033[0;34m"),
      open_cyan_("\033[0;36m"),
      open_green_("\033[0;32m"),
      open_white_("\033[0;37m"),
      open_yellow_("\033[0;33m"),
      open_magenta_("\033[0;35m"),
      open_bold_red_("\033[1;31m"),
      open_bold_blue_("\033[1;34m"),
      open_bold_cyan_("\033[1;36m"),
      open_bold_green_("\033[1;32m"),
      open_bold_white_("\033[1;37m"),
      open_bold_yellow_("\033[1;33m"),
      open_bold_magenta_("\033[1;35m")
  {
  }

  void LqrInfoPrinter::enter()
  {
    std::cout << std::endl;
  }

  std::string LqrInfoPrinter::color_picker(const Color& color)
  {
  	std::string text;
  	switch (color)
  	{
  	  case Color::red : text = open_red_; break;
  	  case Color::blue : text = open_blue_; break;
  	  case Color::cyan : text = open_cyan_; break;
  	  case Color::green : text = open_green_; break;
  	  case Color::white : text = open_white_; break;
  	  case Color::yellow : text = open_yellow_; break;
  	  case Color::magenta : text = open_magenta_; break;
  	  case Color::bold_red : text = open_bold_red_; break;
  	  case Color::bold_blue : text = open_bold_blue_; break;
  	  case Color::bold_cyan : text = open_bold_cyan_; break;
  	  case Color::bold_green : text = open_bold_green_; break;
  	  case Color::bold_white : text = open_bold_white_; break;
  	  case Color::bold_yellow : text = open_bold_yellow_; break;
  	  case Color::bold_magenta : text = open_bold_magenta_; break;
  	}
  	return text;
  }

  void LqrInfoPrinter::printMsg(const std::string& str, const Color& color)
  {
    std::cout << this->color_picker(color) + str + close_;
  }

  void LqrInfoPrinter::printMsg(const std::string& str, std::vector<double>& v_val, std::vector<bool>& v_scientific,
                                const Color& text_color, const Color& num_color )
  {
	std::string text_cstr = this->color_picker(text_color);
	std::string num_cstr  = this->color_picker(num_color);
	std::string text = "";
	std::vector<std::string> v_str, v_prec;

	this->split(str, '%', v_str, v_prec);
	for (int i=0; i<(int)v_str.size(); i++) {
      text = text + text_cstr + v_str[i] + close_;
      if ((int)v_val.size() > i)
        text = text + num_cstr + toString(v_val[i], atoi(v_prec[i].c_str()), v_scientific[i]) + close_;
	}
	std::cout << text;
  }

  void LqrInfoPrinter::split(const std::string& s, char delim,
		  std::vector<std::string>& v_str, std::vector<std::string>& v_prec)
  {
    int i = 0, pos = s.find(delim);
    while (pos != (int)std::string::npos)
    {
      v_str.push_back(s.substr(i, pos-i));
      v_prec.push_back(s.substr(pos+1,1));
      i = ++pos + 1;
      pos = s.find(delim, pos);

      if (pos == (int)std::string::npos)
        v_str.push_back(s.substr(i, s.length()));
    }
  }

  // LqrInfoPrinter class main functions implementations
  void LqrInfoPrinter::initialize(const SolverLqrSetting& stgs)
  {
    stgs_ = std::make_shared<SolverLqrSetting>(stgs);
	prec_ = std::to_string(this->getLqrSetting().get(SolverLqrIntParam_PrecisionDigits));
  }

  void LqrInfoPrinter::display(const MsgLqr& msg, const LqrOptimizationInfo& info)
  {
    switch (msg) {
      case MsgLqr::EXIT_divergence :
      {
        if (this->getLqrSetting().get(SolverLqrIntParam_Verbosity) > 0)
  	      this->printMsg("EXIT: Initial control sequence caused divergence.\n", Color::bold_red);
  	    break;
      }
      case MsgLqr::EXIT_maxiter :
      {
        if (this->getLqrSetting().get(SolverLqrIntParam_Verbosity) > 0)
          this->printMsg("EXIT: Maximum iterations reached.\n", Color::bold_red);
        break;
      }
      case MsgLqr::EXIT_maxlambda :
      {
        if (this->getLqrSetting().get(SolverLqrIntParam_Verbosity) > 1)
          this->printMsg("EXIT: lambda > lambda_max.\n", Color::bold_red);
        break;
      }
      case MsgLqr::SUCC_costchange :
      {
        if (this->getLqrSetting().get(SolverLqrIntParam_Verbosity) > 0)
          this->printMsg("SUCCESS: cost change < tol_fun.\n", Color::bold_green);
        break;
      }
      case MsgLqr::SUCC_gradient :
      {
        if (this->getLqrSetting().get(SolverLqrIntParam_Verbosity) > 0)
          this->printMsg("SUCCESS: gradient norm < tol_grad.\n", Color::bold_green);
        break;
      }
      case MsgLqr::STAT_begin :
      {
 	    if (this->getLqrSetting().get(SolverLqrIntParam_Verbosity) > 0)
 	      this->printMsg("=========== begin optimization ===========\n", Color::bold_blue);
  	    break;
      }
      case MsgLqr::STAT_cholesky :
      {
        if (this->getLqrSetting().get(SolverLqrIntParam_Verbosity) > 2)
          this->printMsg(std::string("  Cholesky failed at timestep ") + std::to_string(info.get(SolverLqrIntParam_BackpassDivergeIteration)) + std::string("\n"), Color::bold_magenta);
    	    break;
      }
      case MsgLqr::STAT_success :
      {
    	    if (this->getLqrSetting().get(SolverLqrIntParam_Verbosity) > 1) {
    	      std::vector<double> v_val = {info.get(SolverLqrIntParam_CurrentIteration)+1.0,
                                       info.get(SolverLqrDoubleParam_Cost),
                                       info.get(SolverLqrDoubleParam_CostChange),
                                       info.get(SolverLqrDoubleParam_ControlGradient),
                                       std::log10(info.get(SolverLqrDoubleParam_CurrentRegularization))};
    	      std::vector<bool> is_scientific = {false, true, true, true, false};
    	      this->printMsg("Iter: %0 cost: %" +prec_+ " reduc: %" +prec_+ " grad: %" +prec_+ " log10lam: %" +prec_+ "\n", v_val, is_scientific);
        }
    	    break;
      }
      case MsgLqr::STAT_rejection : {
   	    if (this->getLqrSetting().get(SolverLqrIntParam_Verbosity) > 1) {
          std::vector<double> v_val = {info.get(SolverLqrIntParam_CurrentIteration)+1.0,
                                       info.get(SolverLqrDoubleParam_ExpectedCost),
                                       info.get(SolverLqrDoubleParam_CostChange),
                                       std::log10(info.get(SolverLqrDoubleParam_CurrentRegularization))};
          std::vector<bool> is_scientific = {false, true, true, false};
          this->printMsg("Iter: %0 REJECTED   expected: %" +prec_+ " actual: %" +prec_+ " log10 lambda: %" +prec_+ "\n", v_val, is_scientific);
   	    }
    	    break;
      }
      case MsgLqr::STAT_summary :
      {
        if (this->getLqrSetting().get(SolverLqrIntParam_Verbosity) > 0) {
          std::vector<double> v_val = {info.get(SolverLqrIntParam_CurrentIteration)+1.0,
                                       info.get(SolverLqrDoubleParam_Cost),
                                       info.get(SolverLqrDoubleParam_ControlGradient)};
          std::vector<bool> is_scientific = {false, true, true};
          this->printMsg("\niterations: %0\n"
        	            "final cost: %" +prec_+ "\n"
        	            "final grad: %" +prec_+ "\n"
        	            ,v_val, is_scientific, Color::bold_yellow);
          this->printMsg("===========  end optimization  ===========\n", Color::bold_blue);
        }
    	    break;
      }
      default:
      {
  	    break;
      }
    }
  }

}
