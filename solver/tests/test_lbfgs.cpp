/**
 * @file TestLbfgs.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-07
 */

#include <gtest/gtest.h>

#include <solver/optimizer/LbfgsSolver.hpp>
#include <test_problems/TestProblem01.hpp>
#include <test_problems/TestProblem02.hpp>
#include <test_problems/TestProblem03.hpp>

#define PRECISION 0.01

  using namespace solver;

  class LbfgsTest : public ::testing::Test
  {
    protected:
      virtual void SetUp() {}
      virtual void TearDown() {}
  };

  template <typename Derived1, typename Derived2>
  void check_matrix(const Derived1& ref, const Derived2& val)
  {
    for (int i=0; i<ref.rows(); i++)
    {
  	  for (int j=0; j<ref.cols(); j++)
  	  {
        EXPECT_NEAR(ref(i,j), val(i,j), 0.01);
  	  }
    }
  }

  TEST_F(LbfgsTest, LbfgsTest01)
  {
    // Reference values
    Eigen::Vector4d ref_x(1.165, 4.312, 4.307, 1.172);
    double ref_objval = 17.6919;

    // Build nonlinear problem
    nlp_test_problems::TestProblem01 toy_problem;

    // Build solver
    LbfgsSolver solver;
    solver.initialize(&toy_problem, 5.);
    solver.verbose() = false;
    solver.opt_params().max_iterations = 10;
    solver.optimize();

    // Retrieve optimal values
    Eigen::VectorXd OptVec = toy_problem.optimalVector();
    double OptVal = toy_problem.optimalValue();

    // Compare optimal values and reference
    check_matrix(OptVec, ref_x);
    EXPECT_NEAR(OptVal, ref_objval, PRECISION);
  }

  TEST_F(LbfgsTest, LbfgsTest02)
  {
    // Reference values
    Eigen::Vector2d ref_x(0.0898418, -0.712656);
    double ref_objval = -1.03163;

    // Build nonlinear problem
    nlp_test_problems::TestProblem02 toy_problem;

    // Build solver
    LbfgsSolver solver;
    solver.initialize(&toy_problem, 0.);
    solver.optimize();

    // Retrieve optimal values
    Eigen::VectorXd OptVec = toy_problem.optimalVector();
    double OptVal = toy_problem.optimalValue();

    // Compare optimal values and reference
    check_matrix(OptVec, ref_x);
    EXPECT_NEAR(OptVal, ref_objval, PRECISION);
  }

  TEST_F(LbfgsTest, LbfgsTest03)
  {
    // Reference values
    Eigen::VectorXd ref_x(100); ref_x.setOnes();
    double ref_objval = 0.0;

    // Build nonlinear problem
    nlp_test_problems::TestProblem03 toy_problem;

    // Build solver
    LbfgsSolver solver;
    solver.initialize(&toy_problem, 0.);
    solver.optimize();

    // Retrieve optimal values
    Eigen::VectorXd OptVec = toy_problem.optimalVector();
    double OptVal = toy_problem.optimalValue();

    // Compare optimal values and reference
    check_matrix(OptVec, ref_x);
    EXPECT_NEAR(OptVal, ref_objval, PRECISION);
  }
