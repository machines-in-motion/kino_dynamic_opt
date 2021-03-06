/**
 * @file test_momentumopt.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#include <gtest/gtest.h>
#include <momentumopt/dynopt/DynamicsOptimizer.hpp>
#include <momentumopt/cntopt/ContactPlanFromFile.hpp>

#define PRECISION 0.01
#define REDUCED_PRECISION 0.06
static const bool display_time_info = false;

using namespace solver;
using namespace momentumopt;

  class DISABLED_MomentumOptTest : public ::testing::Test {};

  class MomentumOptTest : public ::testing::Test
  {
    protected:
      virtual void SetUp() {}
      virtual void TearDown() {}
  };

  void testProblem(const std::string cfg_file, const std::string ref_name, const ExitCode expected_code, const bool& tinfo)
  {
    // load reference data
    std::string data_file = TEST_PATH + std::string("momentumopt_data.yaml");
    Eigen::MatrixXd ref_com, ref_lmom, ref_amom;
    try {
	  YAML::Node ref_cfg = YAML::LoadFile(data_file.c_str());
      YAML::ReadParameter(ref_cfg["solutions"], "com"+ref_name, ref_com);
      YAML::ReadParameter(ref_cfg["solutions"], "lmom"+ref_name, ref_lmom);
      YAML::ReadParameter(ref_cfg["solutions"], "amom"+ref_name, ref_amom);
    } catch (std::runtime_error& e) {
      	std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }

    // define problem configuration
    PlannerSetting planner_setting;
    planner_setting.initialize(cfg_file);

    // define robot initial state
    DynamicsState ini_state;
    ini_state.fillInitialRobotState(cfg_file);

    // define reference dynamic sequence
    KinematicsSequence kin_sequence;
    kin_sequence.resize(planner_setting.get(PlannerIntParam_NumTimesteps),
                        planner_setting.get(PlannerIntParam_NumDofs));

    // define terrain description
    momentumopt::TerrainDescription terrain_description;
    terrain_description.loadFromFile(cfg_file);

    // define contact plan
    ContactPlanFromFile contact_plan;
    contact_plan.initialize(planner_setting);
    contact_plan.optimize(ini_state, terrain_description);

    // optimize motion
    DynamicsOptimizer dyn_optimizer;
    dyn_optimizer.initialize(planner_setting);
    ExitCode exit_code = dyn_optimizer.optimize(ini_state, &contact_plan, kin_sequence);

    if (tinfo) { std::cout << "  Timesteps:" << std::fixed << std::setw(4) << std::setprecision(0) << planner_setting.get(PlannerIntParam_NumTimesteps)
    	                       << "  ActiveEff:" << std::fixed << std::setw(3) << std::setprecision(0) << planner_setting.get(PlannerIntParam_NumActiveEndeffectors)
                           << "  SolveTime:" << std::fixed << std::setw(6) << std::setprecision(0) << dyn_optimizer.solveTime() << " ms"<< std::endl; }

    // check results
    EXPECT_NEAR(static_cast<int>(expected_code), static_cast<int>(exit_code), PRECISION);
    for (int time=0; time<dyn_optimizer.dynamicsSequence().size(); time++)
      for (int id=0; id<3; id++) {
        EXPECT_NEAR(ref_com(time,id) , dyn_optimizer.dynamicsSequence().dynamicsState(time).centerOfMass()[id] , REDUCED_PRECISION);
        EXPECT_NEAR(ref_lmom(time,id), dyn_optimizer.dynamicsSequence().dynamicsState(time).linearMomentum()[id], REDUCED_PRECISION);
        EXPECT_NEAR(ref_amom(time,id), dyn_optimizer.dynamicsSequence().dynamicsState(time).angularMomentum()[id], REDUCED_PRECISION);
      }
  }

  // Testing SoftConstraints Momentum Optimizer with Parallel Interior Point Solver
  TEST_F(MomentumOptTest, test_SoftConstraints_MomentumOptimizer_IPSolver01) {
    testProblem(TEST_PATH+std::string("momopt_demos/cfg_momSc_demo01.yaml"), "MomSc01", ExitCode::Optimal, display_time_info);
  }

  TEST_F(MomentumOptTest, test_SoftConstraints_MomentumOptimizer_IPSolver02) {
    testProblem(TEST_PATH+std::string("momopt_demos/cfg_momSc_demo02.yaml"), "MomSc02", ExitCode::Optimal, display_time_info);
  }

  TEST_F(MomentumOptTest, test_SoftConstraints_MomentumOptimizer_IPSolver03) {
    testProblem(TEST_PATH+std::string("momopt_demos/cfg_momSc_demo03.yaml"), "MomSc03", ExitCode::Optimal, display_time_info);
  }

  // Testing TrustRegion Momentum Optimizer with Interior Point Solver
  TEST_F(MomentumOptTest, test_TrustRegion_MomentumOptimizer_IPSolver01) {
    testProblem(TEST_PATH+std::string("momopt_demos/cfg_momTr_demo01.yaml"), "MomTr01", ExitCode::Optimal, display_time_info);
  }

  TEST_F(DISABLED_MomentumOptTest, test_TrustRegion_MomentumOptimizer_IPSolver02) {
    testProblem(TEST_PATH+std::string("momopt_demos/cfg_momTr_demo02.yaml"), "MomTr02", ExitCode::Optimal, display_time_info);
  }

  TEST_F(MomentumOptTest, test_TrustRegion_MomentumOptimizer_IPSolver03) {
    testProblem(TEST_PATH+std::string("momopt_demos/cfg_momTr_demo03.yaml"), "MomTr03", ExitCode::Optimal, display_time_info);
  }

  // Testing Time Optimizer with Interior Point Solver
  TEST_F(MomentumOptTest, test_TimeMomentumOptimizer_IPSolver01) {
    testProblem(TEST_PATH+std::string("timeopt_demos/cfg_timeopt_demo01.yaml"), "Time01", ExitCode::Optimal, display_time_info);
  }

  TEST_F(MomentumOptTest, test_TimeMomentumOptimizer_IPSolver02) {
  	testProblem(TEST_PATH+std::string("timeopt_demos/cfg_timeopt_demo02.yaml"), "Time02", ExitCode::Optimal, display_time_info);
  }

  TEST_F(MomentumOptTest, test_TimeMomentumOptimizer_IPSolver03) {
  	testProblem(TEST_PATH+std::string("timeopt_demos/cfg_timeopt_demo03.yaml"), "Time03", ExitCode::Optimal, display_time_info);
  }
