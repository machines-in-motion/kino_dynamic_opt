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
