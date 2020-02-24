/**
 * @file PySetting.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-06
 */

#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>

#include <solver/interface/SolverSetting.hpp>

namespace py = pybind11;
using namespace solver;

void init_setting(py::module &m)
{
  py::enum_<ExitCode>(m, "ExitCode")
      .value("Optimal", ExitCode::Optimal, "Problem solved to optimality ")
      .value("OptimalInacc", ExitCode::OptimalInacc, "Problem solved to optimality (Inaccurate)")
      .value("PrimalInf", ExitCode::PrimalInf, "Found certificate of primal infeasibility")
      .value("PrimalInfInacc", ExitCode::PrimalInfInacc, "Found certificate of primal infeasibility (Inaccurate)")
      .value("DualInf", ExitCode::DualInf, "Found certificate of dual infeasibility")
      .value("DualInfInacc", ExitCode::DualInfInacc, "Found certificate of dual infeasibility (Inaccurate)")
      .value("ReachMaxIters", ExitCode::ReachMaxIters, "Algorithm reached maximum number of iterations")
      .value("NotConverged", ExitCode::NotConverged, "Algorithm has not converged yet  ")
      .value("Indeterminate", ExitCode::Indeterminate, "Nothing can be said about the solution")
      .value("PrSearchDirection", ExitCode::PrSearchDirection, "Not reliable search direction ")
      .value("PrSlacksLeaveCone", ExitCode::PrSlacksLeaveCone, "Slack variables lie outside convex cone ")
      .value("PrProjection", ExitCode::PrProjection, "Failed in the projection of cone or linear system")
      .export_values();
}
