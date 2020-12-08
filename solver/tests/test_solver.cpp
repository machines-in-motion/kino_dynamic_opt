/**
 * @file TestSolver.cpp
 * @author A. Domahidi [domahidi@embotech.com]
 * @license (new license) License BSD-3-Clause
 * @copyright Copyright (c) [2012-2015] Automatic Control Lab, ETH Zurich & embotech GmbH, Zurich, Switzerland.
 * @date 2012-2015
 * @brief ECOS - Embedded Conic Solver
 * 
 * Modified to c++ code by New York University and Max Planck Gesellschaft, 2017 
 */


#include <gtest/gtest.h>
#include <yaml_utils/yaml_cpp_fwd.hpp>
#include <solver/interface/Solver.hpp>

#define PRECISION 0.01
#define REDUCED_PRECISION 0.06

using namespace solver;

class SolverTest : public ::testing::Test
{
    protected:
      virtual void SetUp() {}
      virtual void TearDown() {}
};

class ProblemData
{
	public:
	  ProblemData(){}
	  ~ProblemData(){}
	  ProblemData(const std::string cfg_file, bool sparseformat) { this->load_data(cfg_file, sparseformat); }

	  const int& numLeq() const { return nleq_; }
	  const int& numLpc() const { return nlpc_; }
	  const int& numSoc() const { return nsoc_; }
	  const int& numVars() const { return nvars_; }
	  const int& sizeCone() const { return ncone_; }
	  const bool& binProb() const { return binprob_; }
	  const Eigen::VectorXi& sizeSoc() const { return qvec_; }

	  Eigen::VectorXd& b() { return bvec_; }
	  Eigen::VectorXd& c() { return cvec_; }
	  Eigen::VectorXd& h() { return hvec_; }
	  Eigen::VectorXd& xopt() { return xopt_; }
	  Eigen::VectorXi& vartype() { return vartype_; }
	  Eigen::SparseMatrix<double>& A() { return Amat_; }
	  Eigen::SparseMatrix<double>& G() { return Gmat_; }
	  const Eigen::VectorXd& b() const { return bvec_; }
	  const Eigen::VectorXd& c() const { return cvec_; }
	  const Eigen::VectorXd& h() const { return hvec_; }
	  const Eigen::VectorXd& xopt() const { return xopt_; }
	  const Eigen::VectorXi& vartype() const { return vartype_; }
	  const Eigen::SparseMatrix<double>& A() const { return Amat_; }
	  const Eigen::SparseMatrix<double>& G() const { return Gmat_; }

	  void fill_matrix(int rowsize, int colsize,
			           const Eigen::VectorXi& rows, const Eigen::VectorXi& cols,
			           const Eigen::VectorXd& vals, Eigen::SparseMatrix<double>& mat, bool sparseformat)
      {
		mat.resize(colsize, rowsize);
		std::vector<Eigen::Triplet<double>> coeffs;

		if (sparseformat) {
		  for (int i=0; i<colsize; i++)
			for (int j=cols[i]; j<cols[i+1]; j++)
			  coeffs.push_back(Eigen::Triplet<double>(i, rows(j), vals(j)));
		} else {
		  for (int id=0; id<rows.size(); id++)
		    coeffs.push_back(Eigen::Triplet<double>(cols(id), rows(id), vals(id)));
		}
		mat.setFromTriplets(coeffs.begin(),coeffs.end());
		if (!mat.isCompressed()) { mat.makeCompressed(); }
	  }

	  void load_data(const std::string cfg_file, bool sparseformat) {
	    try
	    {
	  	  YAML::Node yaml_file = YAML::LoadFile(cfg_file.c_str());
	  	  YAML::ReadParameter(yaml_file["problem_size"], "nvars"  , nvars_  );
	  	  YAML::ReadParameter(yaml_file["problem_size"], "nleq"   , nleq_   );
	  	  YAML::ReadParameter(yaml_file["problem_size"], "ncone"  , ncone_  );
	  	  YAML::ReadParameter(yaml_file["problem_size"], "nlpc"   , nlpc_   );
	  	  YAML::ReadParameter(yaml_file["problem_size"], "nsoc"   , nsoc_   );
	  	  YAML::ReadParameter(yaml_file["problem_size"], "binprob", binprob_);
	  	  YAML::ReadParameter(yaml_file["problem_size"], "qvec"   , qvec_   );
	  	  YAML::ReadParameter(yaml_file["problem_data"], "Acol"   , Acol_   );
	  	  YAML::ReadParameter(yaml_file["problem_data"], "Arow"   , Arow_   );
	  	  YAML::ReadParameter(yaml_file["problem_data"], "Aval"   , Aval_   );
	  	  YAML::ReadParameter(yaml_file["problem_data"], "Gcol"   , Gcol_   );
	  	  YAML::ReadParameter(yaml_file["problem_data"], "Grow"   , Grow_   );
	  	  YAML::ReadParameter(yaml_file["problem_data"], "Gval"   , Gval_   );
	  	  YAML::ReadParameter(yaml_file["problem_data"], "bvec"   , bvec_   );
	  	  YAML::ReadParameter(yaml_file["problem_data"], "cvec"   , cvec_   );
	  	  YAML::ReadParameter(yaml_file["problem_data"], "hvec"   , hvec_   );
	  	  if (binprob_) {
	  	    YAML::ReadParameter(yaml_file["problem_data"], "xopt", xopt_);
	  	    YAML::ReadParameter(yaml_file["problem_data"], "vartype", vartype_);
	  	  } else {
	  		xopt_.resize(nvars_);    xopt_.setZero();
	  		vartype_.resize(nvars_); vartype_.setZero();
	  	  }
	    }
	    catch (YAML::ParserException &e)
	    {
	      std::cout << e.what() << "\n";
	    }

		this->fill_matrix(nleq_ , nvars_, Arow_, Acol_, Aval_, Amat_, sparseformat);
		this->fill_matrix(ncone_, nvars_, Grow_, Gcol_, Gval_, Gmat_, sparseformat);
	  }

	private:
	  bool binprob_;
	  int nvars_, nleq_, ncone_, nlpc_, nsoc_;
	  Eigen::SparseMatrix<double> Amat_, Gmat_;
	  Eigen::VectorXd Aval_, Gval_, cvec_, bvec_, hvec_, xopt_;
	  Eigen::VectorXi Acol_, Arow_, Gcol_, Grow_, qvec_, vartype_;
};

void buildProblemFromData(Model& model, const ProblemData& data, std::vector<Var>& vars)
{
  // adding variables to problem
	for (int var_id=0; var_id<data.numVars(); var_id++)
	  if (data.vartype()[var_id] == 0) {
	    vars.push_back(model.addVar(VarType::Continuous, 0.0, 1.0, 0.5));
	  } else {
		  vars.push_back(model.addVar(VarType::Binary, 0.0, 1.0, 0.5));
	  }

	// adding linear equality constraints to problem
	if (data.numLeq()>0) {
	  for (int k=0; k<data.A().outerSize(); ++k) {
		LinExpr lhs = -data.b()[k];
		for (Eigen::SparseMatrix<double>::InnerIterator it(data.A(),k); it; ++it)
		  lhs = lhs + LinExpr(vars[it.row()])*it.value();
		model.addLinConstr(lhs, "=", 0.0);
	  }
	}

    // adding linear inequality constraints to problem
	if (data.numLpc()>0) {
	  for (int k=0; k<data.numLpc(); ++k) {
		LinExpr lhs = -data.h()[k];
		for (Eigen::SparseMatrix<double>::InnerIterator it(data.G(),k); it; ++it)
		  lhs += LinExpr(vars[it.row()])*it.value();
		model.addLinConstr(lhs, "<", 0.0);
	  }
	}

	// adding quadratic inequality constraints to problem
	if (data.numSoc()>0) {
	  int row_start = data.numLpc();
	  for (int soc_id=0; soc_id<data.numSoc(); soc_id++) {
		LinExpr lexpr = 0.5*data.h()[data.numLpc()+3*soc_id+2], rhs = 0.0;
		DCPQuadExpr qexpr;
		for (Eigen::SparseMatrix<double>::InnerIterator it(data.G(),row_start+2); it; ++it)
		  lexpr += LinExpr(vars[it.row()])*(-0.5*it.value());
		for (Eigen::SparseMatrix<double>::InnerIterator it(data.G(),row_start+0); it; ++it)
		  rhs = vars[it.row()];
		qexpr.addQuaTerm(1.0, lexpr);
		model.addQuaConstr(qexpr, "<", rhs);
 		row_start += data.sizeSoc()[soc_id];
	  }
	}

	// adding linear objective
	DCPQuadExpr qexpr;
	LinExpr lexpr = 0.0;
	for (int k=0; k<data.numVars(); ++k)
	  lexpr += LinExpr(vars[k])*data.c()[k];
	model.setObjective(qexpr, lexpr);
}

void testProblem(const std::string cfg_file, const ExitCode expected_code, bool sparseformat = false)
{
	Model model;
	std::vector<Var> vars;
	ProblemData data(cfg_file, sparseformat);
	model.configSetting(TEST_PATH+std::string("default_stgs.yaml"));
	model.getSetting().set(SolverBoolParam_Verbose, false);

	buildProblemFromData(model, data, vars);
	ExitCode exit_code = model.optimize();

	if (data.binProb()) {
	  for (int var_id=0; var_id<data.numVars(); var_id++)
	    EXPECT_NEAR(data.xopt()[var_id], vars[var_id].get(SolverDoubleParam_X), PRECISION);
	}
	EXPECT_NEAR(static_cast<int>(expected_code), static_cast<int>(exit_code), PRECISION);
}

// Testing Branch and Bound Solver
TEST_F(SolverTest, BnBSolverTest01)
{
  testProblem(TEST_PATH+std::string("test_BnB_01.yaml"), ExitCode::Optimal);
}

TEST_F(SolverTest, BnBSolverTest02)
{
  testProblem(TEST_PATH+std::string("test_BnB_02.yaml"), ExitCode::Optimal);
}


TEST_F(SolverTest, BnBSolverTest03)
{
  testProblem(TEST_PATH+std::string("test_BnB_03.yaml"), ExitCode::Optimal);
}


TEST_F(SolverTest, BnBSolverTest04)
{
  testProblem(TEST_PATH+std::string("test_BnB_04.yaml"), ExitCode::Optimal);
}


TEST_F(SolverTest, BnBSolverTest05)
{
  testProblem(TEST_PATH+std::string("test_BnB_05.yaml"), ExitCode::Optimal);
}

TEST_F(SolverTest, BnBSolverTest06)
{
  testProblem(TEST_PATH+std::string("test_BnB_06.yaml"), ExitCode::Optimal);
}

TEST_F(SolverTest, BnBSolverTest07)
{
  testProblem(TEST_PATH+std::string("test_BnB_07.yaml"), ExitCode::Optimal);
}


// Testing Interior Point Solver
TEST_F(SolverTest, InteriorPointSolverTest01)
{
  testProblem(TEST_PATH+std::string("test_01.yaml"), ExitCode::Optimal);
}

TEST_F(SolverTest, InteriorPointSolverTest02)
{
  testProblem(TEST_PATH+std::string("test_02.yaml"), ExitCode::Optimal);
}

TEST_F(SolverTest, InteriorPointSolverTest03)
{
  testProblem(TEST_PATH+std::string("test_03.yaml"), ExitCode::Optimal);
}

TEST_F(SolverTest, InteriorPointSolverTest04)
{
  testProblem(TEST_PATH+std::string("test_04.yaml"), ExitCode::Optimal);
}

TEST_F(SolverTest, InteriorPointSolverTest05)
{
  testProblem(TEST_PATH+std::string("test_05.yaml"), ExitCode::Optimal);
}

TEST_F(SolverTest, InteriorPointSolverTest06)
{
  testProblem(TEST_PATH+std::string("test_06.yaml"), ExitCode::Optimal);
}

TEST_F(SolverTest, InteriorPointSolverTest07)
{
  testProblem(TEST_PATH+std::string("test_07.yaml"), ExitCode::Optimal);
}

TEST_F(SolverTest, InteriorPointSolverTest08)
{
  testProblem(TEST_PATH+std::string("test_08.yaml"), ExitCode::Optimal);
}

TEST_F(SolverTest, InteriorPointSolverTest09)
{
  testProblem(TEST_PATH+std::string("test_09.yaml"), ExitCode::Optimal);
}

TEST_F(SolverTest, InteriorPointSolverTest10)
{
  testProblem(TEST_PATH+std::string("test_10.yaml"), ExitCode::Optimal);
}

TEST_F(SolverTest, InteriorPointSolverTest11)
{
  testProblem(TEST_PATH+std::string("test_11.yaml"), ExitCode::Optimal);
}

TEST_F(SolverTest, InteriorPointSolverTest12)
{
  testProblem(TEST_PATH+std::string("test_12.yaml"), ExitCode::PrimalInf);
}

TEST_F(SolverTest, InteriorPointSolverTest13)
{
  testProblem(TEST_PATH+std::string("test_13.yaml"), ExitCode::PrimalInf);
}

TEST_F(SolverTest, InteriorPointSolverTest14)
{
  testProblem(TEST_PATH+std::string("test_14.yaml"), ExitCode::DualInf);
}

TEST_F(SolverTest, InteriorPointSolverTest15)
{
  testProblem(TEST_PATH+std::string("test_15.yaml"), ExitCode::Optimal);
}

TEST_F(SolverTest, InteriorPointSolverTest16)
{
  testProblem(TEST_PATH+std::string("test_16.yaml"), ExitCode::Optimal);
}

TEST_F(SolverTest, InteriorPointSolverTest17)
{
  testProblem(TEST_PATH+std::string("test_17.yaml"), ExitCode::Optimal);
}

TEST_F(SolverTest, InteriorPointSolverTest18)
{
  testProblem(TEST_PATH+std::string("test_18.yaml"), ExitCode::Optimal);
}

