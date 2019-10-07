/**
 * @file Exprs.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-06
 */

#pragma once

#include <vector>
#include <solver/interface/Var.hpp>

namespace solver {

  /**
   * Helper class to ease the construction of a linear expression (e.g. a*x = b)
   */
  class LinExpr
  {
    public:
      LinExpr(double constant=0.0) { constant_ = constant; vars_.clear(); coeffs_.clear(); }
      LinExpr(Var var, double coeff=1.0) { constant_ = 0.0; vars_.push_back(var); coeffs_.push_back(coeff); }

      double getValue() const;
      static bool isClean(const LinExpr& rhs);
      static LinExpr clean(const LinExpr& rhs);
      size_t size() const { return vars_.size(); }
      double& getConstant() { return constant_; }
      const Var& getVar(int i) const { return vars_[i]; }
      double getCoeff(int i) const { return coeffs_[i]; }
      const double& getConstant() const { return constant_; }
      void clear() { constant_ = 0.0; coeffs_.clear(); vars_.clear(); }

      LinExpr operator*(double factor) const;
      LinExpr& operator+=(const LinExpr& rhs);
      LinExpr& operator+=(double a) { *this += LinExpr(a); return *this; }
      LinExpr& operator-=(double a) { *this += LinExpr(-a); return *this; }
      LinExpr& operator-=(const LinExpr& rhs) { *this += rhs*(-1.0); return *this; }
      LinExpr operator/(double a) { LinExpr result = 0.0; result = *this*(1.0/a); return result; }
      LinExpr operator+(double a) { LinExpr result = *this; result += LinExpr(a); return result; }
      LinExpr operator-(double a) { LinExpr result = *this; result += LinExpr(-a); return result; }
      LinExpr operator+(Var var) { LinExpr result = *this; result += LinExpr(var, 1.0); return result; }
      LinExpr operator-(Var var) { LinExpr result = *this; result += LinExpr(var, -1.0); return result; }
      LinExpr operator+(const LinExpr& rhs) const { LinExpr result = *this; result += rhs; return result; }
      LinExpr operator-(const LinExpr& rhs) const { LinExpr result = *this; result += rhs*(-1.0); return result; }
      LinExpr operator=(const LinExpr& rhs) { constant_ = rhs.constant_; vars_ = rhs.vars_; coeffs_ = rhs.coeffs_; return *this; }
      LinExpr& operator*=(double a) { constant_ *= a; for (int i=0; i<(int)vars_.size(); i++) { coeffs_[i] *= a; } return *this; }
      LinExpr& operator/=(double a) { constant_ /= a; for (int i=0; i<(int)vars_.size(); i++) { coeffs_[i] /= a; } return *this; }

      friend LinExpr operator+(Var var) { return LinExpr(var, 1.0); }
      friend LinExpr operator-(Var var) { return LinExpr(var, -1.0); }
      friend LinExpr operator*(Var var, double a) { return LinExpr(var, a); }
      friend LinExpr operator*(double a, Var var) { return LinExpr(var, a); }
      friend LinExpr operator+(Var x, Var y) { return LinExpr(x)+LinExpr(y); }
      friend LinExpr operator-(Var x, Var y) { return LinExpr(x)-LinExpr(y); }
      friend LinExpr operator/(Var var, double a) { return LinExpr(var, 1.0/a); }
      friend LinExpr operator+(double a, Var var) { return LinExpr(a)+LinExpr(var); }
      friend LinExpr operator+(Var var, double a) { return LinExpr(var)+LinExpr(a); }
      friend LinExpr operator-(double a, Var var) { return LinExpr(a)-LinExpr(var); }
      friend LinExpr operator-(Var var, double a) { return LinExpr(var)-LinExpr(a); }
      friend LinExpr operator+(const LinExpr& rhs) { LinExpr result = rhs; return result; }
      friend LinExpr operator-(const LinExpr& rhs) { LinExpr result = LinExpr()-rhs; return result; }
      friend LinExpr operator-(Var var, LinExpr rhs) { LinExpr result = LinExpr(var)-rhs; return result; }
      friend LinExpr operator+(double a, const LinExpr& rhs) { LinExpr result = LinExpr(a)+rhs; return result; }
      friend LinExpr operator-(double a, const LinExpr& rhs) { LinExpr result = LinExpr(a)-rhs; return result; }
      friend LinExpr operator*(double factor, const LinExpr& rhs) { LinExpr result = rhs*factor; return result; }

      friend std::ostream& operator<<(std::ostream &stream, LinExpr expr);

    private:
      double constant_;
      std::vector<Var> vars_;
      std::vector<double> coeffs_;
  };

  LinExpr operator+(Var var);
  LinExpr operator-(Var var);
  LinExpr operator+(Var x, Var y);
  LinExpr operator-(Var x, Var y);
  LinExpr operator*(Var var, double a);
  LinExpr operator*(double a, Var var);
  LinExpr operator/(Var var, double a);
  LinExpr operator+(double a, Var var);
  LinExpr operator+(Var var, double a);
  LinExpr operator-(double a, Var var);
  LinExpr operator-(Var var, double a);
  LinExpr operator+(const LinExpr& rhs);
  LinExpr operator-(const LinExpr& rhs);
  LinExpr operator-(Var var, LinExpr rhs);
  LinExpr operator+(double a, const LinExpr& rhs);
  LinExpr operator-(double a, const LinExpr& rhs);
  LinExpr operator*(double factor, const LinExpr& rhs);
  std::ostream& operator<<(std::ostream &stream, LinExpr expr);

  /**
   * Helper class to ease the construction of a quadratic expression
   * (e.g. sum_i (a_i*x + b_i)^2 + (c*x + d))
   */
  class DCPQuadExpr
  {
    public:
      DCPQuadExpr() : lexpr_(0.0), trust_region_(false), soft_constraint_(false) {}
	  ~DCPQuadExpr(){}

	  double getValue() const;
	  void addLinTerm(const LinExpr& lexpr) { lexpr_ += lexpr; }
	  void clear() { lexpr_ = 0.0; qexpr_.clear(); coeffs_.clear(); extra_vars_.clear(); }
	  void addQuaTerm(double coeff, const LinExpr& lexpr) { if (coeff != 0.0) { qexpr_.push_back(lexpr); coeffs_.push_back(coeff); } }

	  LinExpr& lexpr() { return lexpr_; }
	  std::string& sense() { return sense_; }
	  bool& trustRegion() { return trust_region_; }
	  std::vector<LinExpr>& qexpr() { return qexpr_; }
	  std::vector<double>& coeffs() { return coeffs_; }
	  bool& softConstraint() { return soft_constraint_; }
	  std::vector<Var>& extraVars() { return extra_vars_; }

	  const LinExpr& lexpr() const { return lexpr_; }
	  const std::string& sense() const { return sense_; }
	  const bool& trustRegion() const { return trust_region_; }
	  const std::vector<LinExpr>& qexpr() const { return qexpr_; }
	  const std::vector<double>& coeffs() const { return coeffs_; }
	  const bool& softConstraint() const { return soft_constraint_; }
	  const std::vector<Var>& extraVars() const { return extra_vars_; }
	  const Var& getVar(int qid, int lid) const { return qexpr_[qid].getVar(lid); }

    private:
      LinExpr lexpr_;
      std::string sense_;
      std::vector<double> coeffs_;
      std::vector<LinExpr> qexpr_;
      std::vector<Var> extra_vars_;
      bool trust_region_, soft_constraint_;
  };
}
