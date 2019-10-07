/**
 * @file Model.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-06
 */

#include <iomanip>
#include <iostream>
#include <solver/interface/Model.hpp>

namespace solver {


  ExitCode Model::optimize()
  {
    ExitCode exit_code = ExitCode::Indeterminate;
    if (this->getProblem().numBinaryVariables()>0 &&
        (this->getProblem().numTrustRegions()>0 || this->getProblem().numSoftConstraints()>0)) {
        ncvx_bnb_solver_.initialize(this->getProblem());
        exit_code = ncvx_bnb_solver_.optimize();
        for (int var_id=0; var_id<(int)this->getProblem().problemVariables().size(); var_id++)
          this->getProblem().problemVariables()[var_id]->set(SolverDoubleParam_X, this->getProblem().getSolver().optimalVector().x()[var_id]);

    } else {
      exit_code = this->getProblem().optimize();
    }
    return exit_code;
  }

}
