/**
 * @file EqRoutine.cpp
 * @author A. Domahidi [domahidi@embotech.com]
 * @license (new license) License BSD-3-Clause
 * @copyright Copyright (c) [2012-2015] Automatic Control Lab, ETH Zurich & embotech GmbH, Zurich, Switzerland.
 * @date 2012-2015
 * @brief ECOS - Embedded Conic Solver
 * 
 * Modified to c++ code by New York University and Max Planck Gesellschaft, 2017 
 */

#include <solver/optimizer/EqRoutine.hpp>

namespace solver {

  void EqRoutine::setEquilibration(const Cone& cone, const SolverSetting& stgs, SolverStorage& stg)
  {
    equil_vec_.initialize(cone);
    cone_ = std::make_shared<Cone>(cone);
    stgs_ = std::make_shared<SolverSetting>(stgs);
    this->ruizEquilibration(stg);
  }

  void EqRoutine::ruizEquilibration(SolverStorage& stg)
  {
    Vector equil_tmp;
    equil_vec_.setOnes();
    equil_tmp.initialize(*cone_);

    // iterative equilibration
    for (int iter=0; iter<stgs_->get(SolverIntParam_EquilibrationIters); iter++) {
      equil_tmp.setZero();

      // infinity norms of rows and columns of optimization matrices
      if (stg.Amatrix().nonZeros()>0) { maxRowsCols(equil_tmp.y().data(), equil_tmp.x().data(), stg.Amatrix()); }
      if (stg.Gmatrix().nonZeros()>0) { maxRowsCols(equil_tmp.z().data(), equil_tmp.x().data(), stg.Gmatrix()); }

      // equilibration of second order cones
      for (int i=0; i<cone_->numSoc(); i++) { equil_tmp.zSoc(i).setConstant( equil_tmp.zSoc(i).sum() ); }
      for (int i=0; i<cone_->sizeProb(); i++) { equil_tmp[i] = fabs(equil_tmp[i]) < 1e-6 ? 1.0 : sqrt(equil_tmp[i]); }

      // matrices equilibration
      if (stg.Amatrix().nonZeros()>0) { equilibrateRowsCols(equil_tmp.y().data(), equil_tmp.x().data(), stg.Amatrix()); }
      if (stg.Gmatrix().nonZeros()>0) { equilibrateRowsCols(equil_tmp.z().data(), equil_tmp.x().data(), stg.Gmatrix()); }

      // update equilibration vector
      equil_vec_.array() *= equil_tmp.array();
    }
    stg.cbh().array() /= equil_vec_.array();
  }

  void EqRoutine::maxRowsCols(double *row_vec, double *col_vec, const Eigen::SparseMatrix<double>& mat)
  {
    const int* outPtr = mat.outerIndexPtr();
    for (int col=0; col<mat.cols(); col++) {
      if (outPtr[col+1]-outPtr[col]>0) {
        Eigen::Map<const Eigen::VectorXd> eig_col(&mat.valuePtr()[outPtr[col]], outPtr[col+1]-outPtr[col]);
        col_vec[col] = eig_col.cwiseAbs().maxCoeff();
        for (Eigen::SparseMatrix<double>::InnerIterator it(mat,col); it; ++it)
          row_vec[it.row()] = std::max(fabs(it.value()), row_vec[it.row()]);
      }
    }
  }

  void EqRoutine::equilibrateRowsCols(const double *row_vec, const double *col_vec, Eigen::SparseMatrix<double>& mat)
  {
    for (int k=0; k<mat.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(mat,k); it; ++it)
        it.valueRef() /= (col_vec[it.col()] * row_vec[it.row()]);
  }

  void EqRoutine::unsetEquilibration(SolverStorage& stg)
  {
    if (stg.Amatrix().nonZeros()>0) { unequilibrateRowsCols(equil_vec_.y().data(), equil_vec_.x().data(), stg.Amatrix()); }
    if (stg.Gmatrix().nonZeros()>0) { unequilibrateRowsCols(equil_vec_.z().data(), equil_vec_.x().data(), stg.Gmatrix()); }
    stg.cbh().array() *= equil_vec_.array();
  }

  void EqRoutine::unequilibrateRowsCols(const double *row_vec, const double *col_vec, Eigen::SparseMatrix<double>& mat)
  {
    for (int k=0; k<mat.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(mat,k); it; ++it)
        it.valueRef() *= (row_vec[it.row()] * col_vec[it.col()]);
  }

  void EqRoutine::scaleVariables(OptimizationVector& opt)
  {
    opt.xyz().array() /= (equil_vec_*opt.tau()).array();
    opt.s().array() *= (equil_vec_.z()/opt.tau()).array();
  }

}
