// Copyright (C) 2004, 2009 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: cpp_example.cpp 2005 2011-06-06 12:55:16Z stefan $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#include <iostream>
#include <gtest/gtest.h>

#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>
#include <test_problems/TestProblem00.hpp>

#define PRECISION 0.01
using namespace Ipopt;

  class IpoptTest : public ::testing::Test
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

  TEST_F(IpoptTest, IpoptTest01)
  {
    // Create an nlp instance
    SmartPtr<TNLP> mynlp = new nlp_test_problems::TestProblem00();

    // Create IpoptApplication instance
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

    // Initialize the IpoptApplication and process the options
    app->Options()->SetIntegerValue("print_level", 0);
    ApplicationReturnStatus status = app->Initialize();

    if (status != Solve_Succeeded) {
      std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    }

    status = app->OptimizeTNLP(mynlp);

    if (status == Solve_Succeeded) {
      // Retrieve some statistics about the solve
      Index iter_count = app->Statistics()->IterationCount();
      EXPECT_NEAR(6.0, iter_count, PRECISION);

      Number final_obj = app->Statistics()->FinalObjective();
      EXPECT_NEAR(-4.0, final_obj, PRECISION);
    }
  }
