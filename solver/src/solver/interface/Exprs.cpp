/**
 * @file Exprs.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-06
 */

#include <cmath>
#include <iostream>
#include <solver/interface/Exprs.hpp>
#include <solver/interface/SolverParams.hpp>

namespace solver {

  // Class for Linear Expressions
  double LinExpr::getValue() const {
    double value = constant_;
    for (unsigned int i = 0; i < vars_.size(); i++)
      value += coeffs_[i]*vars_[i].get(SolverDoubleParam_X);
    return value;
  }

  bool LinExpr::isClean(const LinExpr& rhs) {
    for (size_t i=1; i<rhs.size(); i++){
	  for (size_t j=0; j<i; j++) {
        if (rhs.vars_[i].var_storage_->col_no_ == rhs.vars_[j].var_storage_->col_no_)
		  return false;
	  }
	  if (rhs.coeffs_[i] == 0.0)
	    return false;
    }
    return true;
  }

  LinExpr LinExpr::clean(const LinExpr& rhs) {
    LinExpr result;
    result.constant_ = rhs.constant_;

    for (size_t i=0; i<rhs.size(); i++){
	  bool set = false;
	  for (size_t j=0; j<result.size(); j++) {
        if (rhs.vars_[i].var_storage_->col_no_ == result.vars_[j].var_storage_->col_no_) {
		  result.coeffs_[j] += (rhs.coeffs_[i]);
		  set = true;
	    }
	  }
	  if (!set && rhs.coeffs_[i] != 0.0) {
        result.coeffs_.push_back(rhs.coeffs_[i]);
        result.vars_.push_back(rhs.vars_[i]);
	  }
    }
    return result;
  }

  LinExpr LinExpr::operator*(double factor) const {
	if (factor != 0.0) {
	  LinExpr result = *this;
	  result.getConstant() *= factor;
	  for (size_t i=0; i<result.size(); i++)
        result.coeffs_[i] *= factor;
	  return LinExpr::clean(result);
	}
	return LinExpr();
  }

  LinExpr& LinExpr::operator+=(const LinExpr& rhs) {
    this->getConstant() += rhs.getConstant();
    for (size_t i=0; i<rhs.size(); i++) {
    	  this->vars_.push_back(rhs.vars_[i]);
      this->coeffs_.push_back(rhs.coeffs_[i]);
    }
    *this = LinExpr::clean(*this);
	return *this;
  }

  std::ostream& operator<<(std::ostream &stream, LinExpr expr) {
    stream << expr.constant_;
    for (size_t i=0; i<expr.coeffs_.size(); i++)
      stream << " + " << expr.coeffs_[i] << " var[" << expr.vars_[i].get(SolverIntParam_ColNum) << "]" ;
    return stream;
  }

  // Class for Disciplined Convex Quadratic Expressions
  double DCPQuadExpr::getValue() const {
    double value = 0.0;
    for (size_t i=0; i<coeffs_.size(); i++)
      value += coeffs_[i]*std::pow(qexpr_[i].getValue(), 2.0);
    return value;
  }

}
