#include <string>
#include <iostream>
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <yaml_cpp_catkin/yaml_eigen.h>

#include <solver_lqr/SolverLqr.hpp>

class SolverLqrTest : public ::testing::Test
{
  protected:
    virtual void SetUp() {}
    virtual void TearDown() {}
};

using namespace solverlqr;

void check_state_base(const StateBase& S, double scalar)
{
  for (int i=0; i<S.stateVector().size(); i++)
    EXPECT_NEAR(scalar, S.stateVector()[i], 0.01);
}

TEST_F(SolverLqrTest, state_operators_test)
{
  int xdim = 4;
  double lambda = 0.5, ref1 = 1., ref2 = 2.;
  StateBase s1(xdim), s2(xdim), s3(xdim);
  s1.stateVector().setConstant(ref1);
  s2.stateVector().setConstant(ref2);

  // Checking correct initialization
  check_state_base(s1, ref1);
  check_state_base(s2, ref2);

  // Checking += operator
  s1 += s2;
  check_state_base(s1, ref1+ref2);
  check_state_base(s2, ref2);

  // Checking *= operator
  s1 *= lambda;
  s2 *= pow(lambda, 2.);
  check_state_base(s1, lambda*(ref1+ref2));
  check_state_base(s2, pow(lambda,2.)*ref2);

  // Checking + and * operators in different orders
  s3 = (s1 + s2*lambda)*lambda;
  check_state_base(s1, lambda*(ref1+ref2));
  check_state_base(s2, pow(lambda,2.)*ref2);
  check_state_base(s3, lambda*(lambda*(ref1+ref2) + lambda*pow(lambda,2.)*ref2));

  s3 = lambda*(lambda*s2 + s1);
  check_state_base(s1, lambda*(ref1+ref2));
  check_state_base(s2, pow(lambda,2.)*ref2);
  check_state_base(s3, lambda*(lambda*(ref1+ref2) + lambda*pow(lambda,2.)*ref2));

  // Checking assignment operator
  s1 = s3; s2 = s3;
  check_state_base(s1, lambda*(lambda*(ref1+ref2) + lambda*pow(lambda,2.)*ref2));
  check_state_base(s2, lambda*(lambda*(ref1+ref2) + lambda*pow(lambda,2.)*ref2));
  check_state_base(s3, lambda*(lambda*(ref1+ref2) + lambda*pow(lambda,2.)*ref2));
}

void check_control_base(const ControlBase& ctrl, double scalar)
{
  for (int i=0; i<ctrl.feedforward().size(); i++)
    EXPECT_NEAR(scalar, ctrl.feedforward()[i], 0.01);
}

TEST_F(SolverLqrTest, control_operators_test)
{
  int xdim = 4, udim = 2;
  double lambda = 0.5, ref1 = 1., ref2 = 2.;
  ControlBase c1(xdim,udim), c2(xdim,udim), c3(xdim,udim);
  c1.feedforward().setConstant(ref1);  c1.feedback().setConstant(0.0);
  c2.feedforward().setConstant(ref2);  c2.feedback().setConstant(0.0);

  // Checking correct initialization
  check_control_base(c1, ref1);
  check_control_base(c2, ref2);

  // Checking += operator
  c1 += c2;
  check_control_base(c1, ref1+ref2);
  check_control_base(c2, ref2);

  // Checking *= operator
  c1 *= lambda;
  c2 *= pow(lambda, 2.);
  check_control_base(c1, lambda*(ref1+ref2));
  check_control_base(c2, pow(lambda,2.)*ref2);

  // Checking + and * operators in different orders
  c3 = (c1 + c2*lambda)*lambda;
  check_control_base(c1, lambda*(ref1+ref2));
  check_control_base(c2, pow(lambda,2.)*ref2);
  check_control_base(c3, lambda*(lambda*(ref1+ref2) + lambda*pow(lambda,2.)*ref2));

  c3 = lambda*(lambda*c2 + c1);
  check_control_base(c1, lambda*(ref1+ref2));
  check_control_base(c2, pow(lambda,2.)*ref2);
  check_control_base(c3, lambda*(lambda*(ref1+ref2) + lambda*pow(lambda,2.)*ref2));

  // Checking assignment operator
  c1 = c3; c2 = c3;
  check_control_base(c1, lambda*(lambda*(ref1+ref2) + lambda*pow(lambda,2.)*ref2));
  check_control_base(c2, lambda*(lambda*(ref1+ref2) + lambda*pow(lambda,2.)*ref2));
  check_control_base(c3, lambda*(lambda*(ref1+ref2) + lambda*pow(lambda,2.)*ref2));
}

void check_control_sequence(const ControlSequence& ctrl, double ref_ff, double ref_fb)
{
  for (int time=0; time<ctrl.size(); time++) {
    for (int row=0; row<ctrl.control(time).feedback().rows(); row++) {
      EXPECT_NEAR(ref_ff, ctrl.control(time).feedforward()[row], 0.01);
      for (int col=0; col<ctrl.control(time).feedback().cols(); col++)
        EXPECT_NEAR(ref_fb, ctrl.control(time).feedback()(row,col), 0.01);
    }
  }
}

TEST_F(SolverLqrTest, control_sequence_operators_test)
{
  int tdim = 10, xdim = 4, udim = 2;
  double lambda = 0.5, ref1 = 1., ref2 = 2., ref3 = 0.1, ref4 = 0.2;
  ControlSequence c1, c2, c3;
  c1.resize(tdim, xdim, udim);
  c2.resize(tdim, xdim, udim);
  c3.resize(tdim, xdim, udim);

  for (int time=0; time<tdim; time++)
  {
    c1.control(time).feedforward().setConstant(ref1);  c1.control(time).feedback().setConstant(ref3);
    c2.control(time).feedforward().setConstant(ref2);  c2.control(time).feedback().setConstant(ref4);
  }

  // Checking correct initialization
  check_control_sequence(c1, ref1, ref3);
  check_control_sequence(c2, ref2, ref4);

  // Checking += operator
  c1 += c2;
  check_control_sequence(c1, ref1+ref2, ref4);
  check_control_sequence(c2, ref2, ref4);

  // Checking *= operator
  c1 *= lambda;
  c2 *= pow(lambda, 2.);
  check_control_sequence(c1, lambda*(ref1+ref2), ref4);
  check_control_sequence(c2, pow(lambda,2.)*ref2, ref4);

  // Checking + and * operators in different orders
  c3 = (c1 + c2*lambda)*lambda;
  check_control_sequence(c1, lambda*(ref1+ref2), ref4);
  check_control_sequence(c2, pow(lambda,2.)*ref2, ref4);
  check_control_sequence(c3, lambda*(lambda*(ref1+ref2) + lambda*pow(lambda,2.)*ref2), ref4);

  c3 = lambda*(lambda*c2 + c1);
  check_control_sequence(c1, lambda*(ref1+ref2), ref4);
  check_control_sequence(c2, pow(lambda,2.)*ref2, ref4);
  check_control_sequence(c3, lambda*(lambda*(ref1+ref2) + lambda*pow(lambda,2.)*ref2), ref4);

  // Checking assignment operator
  c1 = c3; c2 = c3;
  check_control_sequence(c1, lambda*(lambda*(ref1+ref2) + lambda*pow(lambda,2.)*ref2), ref4);
  check_control_sequence(c2, lambda*(lambda*(ref1+ref2) + lambda*pow(lambda,2.)*ref2), ref4);
  check_control_sequence(c3, lambda*(lambda*(ref1+ref2) + lambda*pow(lambda,2.)*ref2), ref4);
}
