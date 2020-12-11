/**
 * @file TestSolverLqr.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-07
 */

#include <string>
#include <fstream>
#include <iostream>
#include <gtest/gtest.h>
#include <yaml_utils/yaml_eigen.hpp>

#include <solver_lqr/SolverLqr.hpp>
#include <ocp_problems/TwoDofArm.hpp>

using namespace solverlqr;
using namespace ocp_test_problems;

#define PRECISION 0.01
#define REDUCED_PRECISION 2.0

SolverLqrSetting globalSetting;

class SolverLqrTest : public ::testing::Test
{
  protected:
    virtual void SetUp() {}
    virtual void TearDown() {}
};

void check_matrix(const Eigen::MatrixXd& ref_m, const Eigen::MatrixXd& m)
{
  // check matrices are equal
  for (int row_id=0; row_id<ref_m.rows(); row_id++)
    for (int col_id=0; col_id<ref_m.cols(); col_id++)
      EXPECT_NEAR(ref_m(row_id,col_id), m(row_id,col_id), REDUCED_PRECISION);
}

void optimize_system(const std::string& filename, const std::string& ref_name, OcpBase* ocp, bool randomize_ctrl=false)
{
  // load reference data
  std::string data_file = TEST_PATH + std::string("rscontrol_data.yaml");
  Eigen::MatrixXd ref_states, ref_ctrlff, ref_ctrlfb;
  try {
    YAML::Node ref_cfg = YAML::LoadFile(data_file.c_str());
    ref_states = ref_cfg["solutions"]["states_"+ref_name].as<Eigen::MatrixXd>();
    ref_ctrlff = ref_cfg["solutions"]["ctrlff_"+ref_name].as<Eigen::MatrixXd>();
    ref_ctrlfb = ref_cfg["solutions"]["ctrlfb_"+ref_name].as<Eigen::MatrixXd>();
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter at file: [" << __FILE__ << "]" << std::endl;
  }

  ///////////////////////////////////
  // Start of Problem Optimization //
  ///////////////////////////////////

  // instantiate and initialize an LqrSetting
  std::string cfg_file = TEST_PATH + filename;
  globalSetting.initialize(cfg_file);

  // initialize some variables
  int tdim = globalSetting.get(SolverLqrIntParam_TimeDimension);
  int xdim = globalSetting.get(SolverLqrIntParam_StateDimension);
  int udim = globalSetting.get(SolverLqrIntParam_ControlDimension);
  Eigen::VectorXd xini = globalSetting.get(SolverLqrVectorParam_InitialState);

  // build state and control sequences
  StateBase ini_state(xdim);  ini_state.stateVector() = xini;
  ControlSequence controls(tdim, xdim, udim);
  if (randomize_ctrl) { srand(0); controls.setRandom(0.01); }

  // create instances, initialize and configure
  ocp->initialize(globalSetting);
  ocp->controlSeq() = controls;
  ocp->stateSeq().state(0) = ini_state;

  // instantiate and use algorithm
  SolverLqr lqr;
  lqr.initialize(ocp, globalSetting);
  lqr.optimize();

  /////////////////////////////////
  // End of Problem Optimization //
  /////////////////////////////////

  // check correctness of the solution
  for (int row_id=0; row_id<ocp->tdim()+1; row_id++)
    for (int col_id=0; col_id<ocp->xdim(); col_id++)
      EXPECT_NEAR(ref_states(row_id,col_id), ocp->stateSeq().state(row_id).stateVector()[col_id], REDUCED_PRECISION);

  for (int row_id=0; row_id<ocp->tdim(); row_id++)
    for (int col_id=0; col_id<ocp->udim(); col_id++)
      EXPECT_NEAR(ref_ctrlff(row_id,col_id), ocp->controlSeq().control(row_id).feedforward()[col_id], REDUCED_PRECISION);

  for (int time_id=0; time_id<ocp->tdim(); time_id++)
	for (int row_id=0; row_id<ocp->udim(); row_id++)
      for (int col_id=0; col_id<ocp->xdim(); col_id++)
        EXPECT_NEAR(ref_ctrlfb(time_id,row_id*ocp->xdim()+col_id), ocp->controlSeq().control(time_id).feedback()(row_id,col_id), REDUCED_PRECISION);
}

// LQR tests

TEST_F(SolverLqrTest, Test_Demo01_TwoDofArm)
{
  TwoDofArm finger;
  optimize_system("test_cfg_files/cfg_demo01.yaml", "01A", &finger);
}
