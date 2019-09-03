/*
 * Copyright [2019] Max Planck Society. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <solver/interface/Solver.hpp>

#define PRECISION 0.01
#define REDUCED_PRECISION 0.06

using namespace rt_solver;

  class RtSolverTest : public ::testing::Test
  {
    protected:
      virtual void SetUp() {}
      virtual void TearDown() {}
  };

  static const int Num_OptVars = 4;
  static const int Max_Eq_Rows = 10;
  static const int Max_Ineq_Rows = 10;

  /*
   * Easy way to formulate a hierarchical problem
   */

  TEST_F(RtSolverTest, HQPSolverTest01)
  {
    // default parameters are used if not otherwise specified
    YAML::Node hqp_solver_pars;

    // instance of model
    RtModel<Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars> rt_model(hqp_solver_pars);

    // compose problem
    rt_model.clean();
    rt_model.addLinConstr( rt_model.getVar(0), ">", 1.0, 0);
    rt_model.addLinConstr( rt_model.getVar(0), ">", 1.0, 0);

    rt_model.addLinConstr( rt_model.getVar(0), "=", -2.0, 1);
    rt_model.addLinConstr( rt_model.getVar(1), "=",  3.0, 1);

    // find a solution
    rt_model.initialize();
    if (!rt_model.solve())
      std::cout << "solution invalid" << std::endl;

    // test final solution
    Eigen::Vector4d solution; solution << 1.0, 3.0, 0.0, 0.0;
    for (int id=0; id<solution.size(); id++)
      EXPECT_NEAR(solution[id], rt_model.hqp_solver_.solution()[id], PRECISION);
  }

  TEST_F(RtSolverTest, HQPSolverTest02)
  {
    // default parameters are used if not otherwise specified
    YAML::Node hqp_solver_pars;

    // instance of model
    RtModel<Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars> rt_model(hqp_solver_pars);

    // compose problem
    rt_model.clean();
    rt_model.addLinConstr(-rt_model.getVar(0)+rt_model.getVar(1), ">", 1.0, 0);
    rt_model.addLinConstr( rt_model.getVar(0)+rt_model.getVar(1), ">", 1.0, 0);

    rt_model.addLinConstr( rt_model.getVar(0), "=", 2.0, 1);
    rt_model.addLinConstr( rt_model.getVar(1), "=", 5.0, 1);

    // find a solution
    rt_model.initialize();
    if (!rt_model.solve())
      std::cout << "solution invalid" << std::endl;

    // test final solution
    Eigen::Vector4d solution; solution << 2.0, 5.0, 0.0, 0.0;
    for (int id=0; id<solution.size(); id++)
      EXPECT_NEAR(solution[id], rt_model.hqp_solver_.solution()[id], PRECISION);
  }


  /*
   * Typical alternative to formulate the hierarchical problem
   * using the RtHQPCost class and methods
   */

  template<int Max_Ineq_Rows, int Max_Eq_Rows, int Num_OptVars>
  class ConeConstraints : public rt_solver::RtHQPCost
  {
    public:
      typedef RtModel<Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars> RtModel_;

    public:
      ConeConstraints(RtModel_& prob_composer)
        : prob_composer_(&prob_composer)
      {
        vec_ = Eigen::Vector2d(1.0, 1.0);
        Eigen::Matrix<double, 2, 4> helper;
        helper <<  1.0, -1.0,  0.0,  0.0,
                  -1.0, -1.0,  0.0,  0.0;
        mat_ = helper;
        weight_.setIdentity();
        ranks_ = Eigen::Matrix<int,2,1>::Zero();
      }
      virtual ~ConeConstraints() {}

      void updateAfterSolutionFound() {}
      int maxRank() const { return ranks_.maxCoeff(); }
      void addCostToHierarchy(int rank) const
      {
        prob_composer_->hqp_solver_.applyIneqSlacks() = true;
        prob_composer_->template appendRowsOfRank<2>(rank, ranks_, mat_, vec_, false, 0);
      }

    private:
      RtModel_ *prob_composer_;

      rt_solver::RtVector<2>::d vec_;
      rt_solver::RtMatrix<2,4>::d mat_;
      rt_solver::RtVector<2>::i ranks_;
      Eigen::Matrix<double, 2, 2> weight_;
  };

  template<int Max_Ineq_Rows, int Max_Eq_Rows, int Num_OptVars>
  class HyperConstraints : public rt_solver::RtHQPCost
  {
    public:
      typedef RtModel<Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars> RtModel_;

    public:
      HyperConstraints(RtModel_& prob_composer)
        : prob_composer_(&prob_composer)
      {
        weight_.setIdentity();
        vec_ = Eigen::Vector2d(-2.0, -5.0);
        Eigen::Matrix<double, 2, 4> helper;
        helper <<  1.0,  0.0,  0.0,  0.0,
                   0.0,  1.0,  0.0,  0.0;
        mat_ = helper;
        ranks_ = Eigen::Matrix<int,2,1>::Ones();
      }
      virtual ~HyperConstraints() {}

      void updateAfterSolutionFound() {}
      int maxRank() const { return ranks_.maxCoeff(); }
      void addCostToHierarchy(int rank) const
      {
        prob_composer_->template appendRowsOfRank<2>(rank, ranks_, mat_, vec_, true, 0);
      }

    private:
      RtModel_ *prob_composer_;

      rt_solver::RtVector<2>::d vec_;
      rt_solver::RtMatrix<2,4>::d mat_;
      rt_solver::RtVector<2>::i ranks_;
      Eigen::Matrix<double, 2, 2> weight_;
  };

  // Testing real-time hqp solver
  TEST_F(RtSolverTest, HQPSolverTest03)
  {
    // model parameters
    YAML::Node hqp_solver_pars;
    hqp_solver_pars["ineq_relax"] = 1.e-6;
    hqp_solver_pars["hsol_max_eq_cond"] = 1.e8;
    hqp_solver_pars["psd_hessian_diag"] = 1.e-8;

    // instance of model
    RtModel<Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars> rt_model(hqp_solver_pars);

    // instances of sub-cost composers
    ConeConstraints<Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars> cons_A(rt_model);
    HyperConstraints<Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars> cons_B(rt_model);

    // adding costs to rt model
    rt_model.subCostComposers().push_back(static_cast<RtHQPCost*>(&cons_A));
    rt_model.subCostComposers().push_back(static_cast<RtHQPCost*>(&cons_B));

    // find a solution
    rt_model.initialize();
    if (!rt_model.solve())
      std::cout << "solution invalid" << std::endl;

    // test final solution
    Eigen::Vector4d solution; solution << 2.0, 5.0, 0.0, 0.0;
    for (int id=0; id<solution.size(); id++)
      EXPECT_NEAR(solution[id], rt_model.hqp_solver_.solution()[id], PRECISION);
  }
