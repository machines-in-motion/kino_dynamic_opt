/**
 * @file Solver.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-06
 */

#pragma once

#ifdef USE_GUROBI
#include <gurobi_c++.h>
#endif

#ifdef USE_IPOPT
#include <IpSolveStatistics.hpp>
#include <IpIpoptApplication.hpp>
#endif

#include <solver/interface/Var.hpp>
#include <solver/interface/Exprs.hpp>
#include <solver/interface/Model.hpp>
#include <solver/interface/OptVar.hpp>
#include <solver/rt_optimizer/RtModel.h>
#include <solver/interface/SolverParams.hpp>
#include <solver/interface/SolverSetting.hpp>
