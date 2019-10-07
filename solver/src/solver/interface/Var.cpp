/**
 * @file Var.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-06
 */

#include <iostream>
#include <stdexcept>
#include <solver/interface/Var.hpp>

namespace solver {

  Var::Var(int col_no, const VarType& type, double lb, double ub, double guess)
  {
    var_storage_.reset( new VarStorage() );
    var_storage_->lb_ = lb;
    var_storage_->ub_ = ub;
    var_storage_->type_ = type;
    var_storage_->value_ = guess;
    var_storage_->guess_ = guess;
    var_storage_->col_no_ = col_no;
  }

  int Var::get(SolverIntParam param) const
  {
    int value;
    if (var_storage_ == nullptr)
      throw std::runtime_error("Variable not initialized");

    switch (param) {
      case SolverIntParam_ColNum : { value = this->var_storage_->col_no_; break; }
      default: { throw std::runtime_error("Var::get SolverIntParam"); break; }
    }
    return value;
  }

  void Var::set(SolverIntParam param, int value)
  {
    switch (param) {
      case SolverIntParam_ColNum : { this->var_storage_->col_no_ = value; break; }
      default: { throw std::runtime_error("Var::set SolverIntParam"); break; }
    }
  }

  double Var::get(SolverDoubleParam param) const
  {
    double value;
    if (var_storage_ == nullptr)
      throw std::runtime_error("Variable not initialized");

    switch (param) {
      case SolverDoubleParam_X : { value = this->var_storage_->value_; break; }
      case SolverDoubleParam_LB : { value = this->var_storage_->lb_; break; }
      case SolverDoubleParam_UB : { value = this->var_storage_->ub_; break; }
      case SolverDoubleParam_Guess : { value = this->var_storage_->guess_; break; }
      default: { throw std::runtime_error("Var::get SolverDoubleParam"); break; }
    }
    return value;
  }

  void Var::set(SolverDoubleParam param, double value)
  {
    switch (param) {
      case SolverDoubleParam_X : { this->var_storage_->value_ = value; break; }
      case SolverDoubleParam_LB : { this->var_storage_->lb_ = value; break; }
      case SolverDoubleParam_UB : { this->var_storage_->ub_ = value; break; }
      case SolverDoubleParam_Guess : { this->var_storage_->guess_ = value; break; }
      default: { throw std::runtime_error("Var::set SolverDoubleParam"); break; }
    }
  }

}
