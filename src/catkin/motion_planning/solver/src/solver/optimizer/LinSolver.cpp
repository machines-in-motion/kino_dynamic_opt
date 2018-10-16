/*
 *
 * ECOS - Embedded Conic Solver
 * Copyright (C) [2012-2015] A. Domahidi [domahidi@embotech.com],
 * Automatic Control Lab, ETH Zurich & embotech GmbH, Zurich, Switzerland.
 *
 * Copyright [2017] Max Planck Society. All rights reserved.
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

#include <solver/optimizer/LinSolver.hpp>

namespace solver {

  void LinSolver::resizeProblemData()
  {
    int psize = this->getCone().extSizeProb();

    Pe_.resize(psize);
    perm_.resize(psize);
    permX_.resize(psize);
    permdX_.resize(psize);
    invPerm_.resize(psize);
    kkt_.resize(psize,psize);
    permKkt_.resize(psize,psize);
    Gdx_.initialize(this->getCone());
    sign_.initialize(this->getCone());
    permSign_.initialize(this->getCone());
  }

  void LinSolver::buildProblem()
  {
    int n = this->getCone().numVars();
    int p = this->getCone().numLeq();

    // Vector for regularization
    sign_.x().setOnes();
    sign_.y().setConstant(-1.0);
    sign_.z().setConstant(-1.0);
    for (int i=0; i<this->getCone().numSoc(); i++)
      sign_.zSoc(i)[this->getCone().sizeSoc(i)+1] =  1.0;

    // Building KKT matrix
    std::vector<Eigen::Triplet<double>> coeffs;
    static_regularization_ = this->getSetting().get(SolverDoubleParam_StaticRegularization);

    for (int id=0; id<n; id++)                 // KKT matrix (1,1)
      coeffs.push_back(Eigen::Triplet<double>(id,id,static_regularization_));

    for (int id=0; id<this->getStorage().Atmatrix().outerSize(); id++) {  // KKT matrix (1,2) A'
      for (Eigen::SparseMatrix<double>::InnerIterator it(this->getStorage().Atmatrix(),id); it; ++it)
        coeffs.push_back(Eigen::Triplet<double>(it.row(),n+it.col(),it.value()));
      coeffs.push_back(Eigen::Triplet<double>(n+id,n+id,-static_regularization_));
    }

    for (int id=0; id<this->getCone().sizeLpc(); id++) {  // KKT matrix (1,3)-(3,3) G' and -Weights
      for (Eigen::SparseMatrix<double>::InnerIterator it(this->getStorage().Gtmatrix(),id); it; ++it)
        coeffs.push_back(Eigen::Triplet<double>(it.row(),n+p+it.col(),it.value()));
      coeffs.push_back(Eigen::Triplet<double>(n+p+id,n+p+id,-1.0));
      this->getCone().indexLpc(id) = n+p+this->getStorage().Atmatrix().nonZeros()+this->getStorage().Gtmatrix().leftCols(id+1).nonZeros()+id;
    }

    int k = n+p+this->getStorage().Atmatrix().nonZeros()+this->getStorage().Gtmatrix().leftCols(this->getCone().sizeLpc()).nonZeros()+this->getCone().sizeLpc();

    for (int l=0; l<this->getCone().numSoc(); l++) {
      for (int id=0; id<this->getCone().sizeSoc(l); id++) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(this->getStorage().Gtmatrix(),this->getCone().startSoc(l)+id); it; ++it)
          coeffs.push_back(Eigen::Triplet<double>(it.row(),n+p+2*l+it.col(),it.value()));
        coeffs.push_back(Eigen::Triplet<double>(n+p+this->getCone().startSoc(l)+2*l+id,n+p+this->getCone().startSoc(l)+2*l+id,-1.0));
        this->getCone().soc(l).indexSoc(id) = k+this->getStorage().Gtmatrix().col(this->getCone().startSoc(l)+id).nonZeros();

        k += this->getStorage().Gtmatrix().col(this->getCone().startSoc(l)+id).nonZeros()+1;
      }
      k += 2*this->getCone().sizeSoc(l)+1;

      for (int id=1; id<this->getCone().sizeSoc(l); id++)
        coeffs.push_back(Eigen::Triplet<double>(n+p+this->getCone().startSoc(l) + 2*l + id,n+p+this->getCone().startSoc(l)+2*l+this->getCone().sizeSoc(l), 0.0));
      coeffs.push_back(Eigen::Triplet<double>(n+p+this->getCone().startSoc(l)+2*l+this->getCone().sizeSoc(l),n+p+this->getCone().startSoc(l)+2*l+this->getCone().sizeSoc(l),-1.0));

      for (int id=0; id<this->getCone().sizeSoc(l); id++)
        coeffs.push_back(Eigen::Triplet<double>(n+p+this->getCone().startSoc(l)+2*l+id,n+p+this->getCone().startSoc(l)+2*l+this->getCone().sizeSoc(l)+1,0.0));
      coeffs.push_back(Eigen::Triplet<double>(n+p+this->getCone().startSoc(l)+2*l+this->getCone().sizeSoc(l)+1,n+p+this->getCone().startSoc(l)+2*l+this->getCone().sizeSoc(l)+1,1.0));
    }

    kkt_.setFromTriplets(coeffs.begin(), coeffs.end());
    if (!kkt_.isCompressed()) { kkt_.makeCompressed(); }
  }

  void LinSolver::findPermutation()
  {
    // find permutation and inverse permutation
    Eigen::AMDOrdering<int> ordering;
    ordering(kkt_, perm_);
    invPerm_ = perm_.inverse();

    // permute quantities
    for (int i=0; i<this->getCone().extSizeProb(); i++) { permSign_[i] = sign_[perm_.indices()[i]]; }
    permKkt_.selfadjointView<Eigen::Upper>() = kkt_.selfadjointView<Eigen::Upper>().twistedBy(invPerm_);
  }

  void LinSolver::symbolicFactorization()
  {
    this->getCholesky().analyzePattern(permKkt_, this->getSetting());
  }

  FactStatus LinSolver::numericFactorization()
  {
    int status = this->getCholesky().factorize(this->permKkt_, this->permSign_);
    return (status == this->permKkt_.cols() ? FactStatus::Optimal : FactStatus::Failure);
  }

  int LinSolver::solve(const Eigen::Ref<const Eigen::VectorXd>& permB, OptimizationVector& searchDir, bool is_initialization)
  {
    int numRefs;
    int nK = this->getCone().extSizeProb();
    int mext = this->getCone().extSizeCone();

    double* Gdx = Gdx_.data();
    Eigen::VectorXd permdZ(mext);
    int* Pinv = this->invPerm_.indices().data();
    ExtendedVector err; err.initialize(this->getCone());
    double errNorm_cur, errNorm_prev = SolverSetting::nan;
    double errThresh = (1.0 + (nK>0 ? permB.lpNorm<Eigen::Infinity>() : 0.0 ))*this->getSetting().get(SolverDoubleParam_LinearSystemAccuracy);

    double* ez = err.z().data();
    double* dz = searchDir.z().data();

    // solve perturbed linear system
    this->getCholesky().solve(permB, permX_.data());

    // iterative refinement due to regularization to KKT matrix factorization
    for (numRefs=0; numRefs <= this->getSetting().get(SolverIntParam_NumIterRefinementsLinSolve); numRefs++)
    {
      this->getCone().unpermuteSolution(invPerm_, permX_, searchDir, permdZ);
      for (int i=0; i<nK; i++) { err[i] = permB[Pinv[i]]; }

      // error_x = b_x - (Is dx + A' dy + G' dz)
      matrixTransposeTimesVector(this->getStorage().Amatrix(), searchDir.y(), err.x(), false, false);
      matrixTransposeTimesVector(this->getStorage().Gmatrix(), searchDir.z(), err.x(), false, false);
      err.x() -= static_regularization_*searchDir.x();

      // error_y = b_y - (A dx - Is dy)
      if (this->getStorage().Amatrix().nonZeros()>0) {
        matrixTransposeTimesVector(this->getStorage().Atmatrix(), searchDir.x(), err.y(), false, false);
        err.y() += static_regularization_*searchDir.y();
      }

      // error_z = b_z - (G dx +(Is+W2) dz)
      matrixTransposeTimesVector(this->getStorage().Gtmatrix(), searchDir.x(), Gdx_, true, true);
      err.zLpc() += -Gdx_.zLpc() + static_regularization_*searchDir.zLpc();
      for (int i=0; i<this->getCone().numSoc(); i++) {
        for (int j=0; j<this->getCone().sizeSoc(i)-1; j++)
          ez[this->getCone().startSoc(i)+2*i+j] += -Gdx[this->getCone().startSoc(i)+j] + static_regularization_*dz[this->getCone().startSoc(i)+j];
        ez[this->getCone().startSoc(i)+2*i+this->getCone().sizeSoc(i)-1] -= Gdx[this->getCone().startSoc(i)+this->getCone().sizeSoc(i)-1] + static_regularization_*dz[this->getCone().startSoc(i)+this->getCone().sizeSoc(i)-1];
      }
      if (is_initialization) { err.z() += permdZ; }
      else { this->getCone().conicNTScaling2(permdZ, err.z()); }

      // progress checks
      errNorm_cur = err.size()>0 ? err.lpNorm<Eigen::Infinity>() : 0.0;
      if (numRefs>0 && errNorm_cur>errNorm_prev ) { permX_ -= permdX_; numRefs--; break; }
      if (numRefs==this->getSetting().get(SolverIntParam_NumIterRefinementsLinSolve) || (errNorm_cur<errThresh) || (numRefs>0 && errNorm_prev<this->getSetting().get(SolverDoubleParam_ErrorReductionFactor)*errNorm_cur)) { break; }
      errNorm_prev = errNorm_cur;

      // solve and add refinement to permX
      for (int i=0; i<nK; i++) { Pe_[Pinv[i]] = err[i]; }
      this->getCholesky().solve(Pe_, permdX_.data());
      permX_ += permdX_;
    }

    // store solution within search direction
    this->getCone().unpermuteSolution(invPerm_, permX_, searchDir);
    return numRefs;
  }

  void LinSolver::initialize(Cone& cone, SolverSetting& setting, SolverStorage& storage)
  {
    cone_ = &cone;
    storage_ = &storage;
    setting_ = &setting;

    resizeProblemData();
    buildProblem();
    findPermutation();
    symbolicFactorization();

    // update indices of NT scalings to access them directly in permuted matrix
    Eigen::VectorXi index = Eigen::Map<Eigen::VectorXi>(permKkt_.outerIndexPtr(), this->getCone().extSizeProb()+1);
    permK_.resize(kkt_.nonZeros());
    int nnz = 0;
    for (int j=0; j<kkt_.outerSize(); j++)
      for (Eigen::SparseMatrix<double>::InnerIterator it(kkt_,j); it; ++it)
        if (it.row()<=it.col()) {
          permK_.indices()[nnz] = index[invPerm_.indices()[it.row()] > invPerm_.indices()[it.col()] ?
                                        invPerm_.indices()[it.row()] : invPerm_.indices()[it.col()]]++;
          nnz += 1;
        }

    for (int id=0; id<this->getCone().sizeLpc(); id++)
      this->getCone().indexLpc(id) = permK_.indices()[this->getCone().indexLpc(id)];

    for (int i=0; i<this->getCone().numSoc(); i++) {
      int j=1;
      for (int k=0; k<2*this->getCone().sizeSoc(i)+1; k++)
        this->getCone().soc(i).indexSoc(this->getCone().sizeSoc(i)+k) = permK_.indices()[this->getCone().soc(i).indexSoc(this->getCone().sizeSoc(i)-1)+j++];
      for (int k=0; k<this->getCone().sizeSoc(i); k++)
        this->getCone().soc(i).indexSoc(k) = permK_.indices()[this->getCone().soc(i).indexSoc(k)];
    }
  }

  void LinSolver::initializeMatrix()
  {
    double* value = permKkt_.valuePtr();

    // Linear cone
    for (int i=0; i<this->getCone().sizeLpc(); i++)
      value[this->getCone().indexLpc(i)] = -1.0;

    // Second order cone
    for (int i=0; i<this->getCone().numSoc(); i++) {
      value[this->getCone().soc(i).indexSoc(0)] = -1.0;
      for (int k=1; k<this->getCone().sizeSoc(i); k++)
        value[this->getCone().soc(i).indexSoc(k)] = -1.0;

      for (int k=0; k<this->getCone().sizeSoc(i)-1; k++)
        value[this->getCone().soc(i).indexSoc(this->getCone().sizeSoc(i)+k)] = 0.0;
      value[this->getCone().soc(i).indexSoc(2*this->getCone().sizeSoc(i)-1)] = -1.0;

      value[this->getCone().soc(i).indexSoc(2*this->getCone().sizeSoc(i))] = 0.0;
      for (int k=0; k<this->getCone().sizeSoc(i)-1; k++)
        value[this->getCone().soc(i).indexSoc(2*this->getCone().sizeSoc(i)+1+k)] = 0.0;
      value[this->getCone().soc(i).indexSoc(3*this->getCone().sizeSoc(i))] = +1.0;
    }
  }

  void LinSolver::updateMatrix()
  {
    static_regularization_ = this->getSetting().get(SolverDoubleParam_StaticRegularization);
    int conesize;
    double eta_square, *scaling_soc;
    double* value = permKkt_.valuePtr();

    // Linear cone
    for (int i=0; i<this->getCone().sizeLpc(); i++)
      value[this->getCone().indexLpc(i)] = -this->getCone().sqScalingLpc(i) - static_regularization_;

    // Second order cone
    for (int i=0; i<this->getCone().numSoc(); i++) {
      conesize = this->getCone().sizeSoc(i);
      eta_square = this->getCone().soc(i).etaSquare();
      scaling_soc = this->getCone().soc(i).scalingSoc().data();

      value[this->getCone().soc(i).indexSoc(0)] = -eta_square * this->getCone().soc(i).d1() - static_regularization_;
      for (int k=1; k<conesize; k++)
        value[this->getCone().soc(i).indexSoc(k)] = -eta_square - static_regularization_;

      for (int k=0; k<conesize-1; k++)
        value[this->getCone().soc(i).indexSoc(conesize+k)] = -eta_square * this->getCone().soc(i).v1() * scaling_soc[k+1];
      value[this->getCone().soc(i).indexSoc(2*conesize-1)] = -eta_square;

      value[this->getCone().soc(i).indexSoc(2*conesize)] = -eta_square * this->getCone().soc(i).u0();
      for (int k=0; k<conesize-1; k++)
        value[this->getCone().soc(i).indexSoc(2*conesize+1+k)] = -eta_square * this->getCone().soc(i).u1() * scaling_soc[k+1];
      value[this->getCone().soc(i).indexSoc(3*conesize)] = +eta_square + static_regularization_;
    }
  }

  void LinSolver::matrixTransposeTimesVector(const Eigen::SparseMatrix<double>& A,
                                             const Eigen::Ref<const Eigen::VectorXd>& eig_x,
                                             Eigen::Ref<Eigen::VectorXd> eig_y,
                                             bool add, bool is_new)
  {
    double ylocal;
    int row, inner_start, inner_end;
    if (is_new) { eig_y.setZero(); }

    double* y = eig_y.data();
    const double* x = eig_x.data();
    const double* valPtr = A.valuePtr();
    const int* outPtr = A.outerIndexPtr();
    const int* innPtr = A.innerIndexPtr();

    if (add) {
      for (int col=0; col<A.cols(); col++) {
        ylocal = y[col];
        inner_start = outPtr[col];
        inner_end   = outPtr[col+1];
        if (inner_end>inner_start) {
          for (row=inner_start; row<inner_end; row++)
          ylocal += valPtr[row]*x[innPtr[row]];
        }
        y[col] = ylocal;
      }
    } else {
      for (int col=0; col<A.cols(); col++) {
        ylocal = y[col];
        inner_start = outPtr[col];
        inner_end   = outPtr[col+1];
        if (inner_end>inner_start) {
          for (row=inner_start; row<inner_end; row++)
            ylocal -= valPtr[row]*x[innPtr[row]];
        }
        y[col] = ylocal;
      }
    }
  }

}
