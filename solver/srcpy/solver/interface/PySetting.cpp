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
      .value("Optimal", ExitCode::Optimal)
      .value("OptimalInacc", ExitCode::OptimalInacc)
      .value("PrimalInf", ExitCode::PrimalInf)
      .value("PrimalInfInacc", ExitCode::PrimalInfInacc)
      .value("DualInf", ExitCode::DualInf)
      .value("DualInfInacc", ExitCode::DualInfInacc)
      .value("ReachMaxIters", ExitCode::ReachMaxIters)
      .value("NotConverged", ExitCode::NotConverged)
      .value("Indeterminate", ExitCode::Indeterminate)
      .value("PrSearchDirection", ExitCode::PrSearchDirection)
      .value("PrSlacksLeaveCone", ExitCode::PrSlacksLeaveCone)
      .value("PrProjection", ExitCode::PrProjection)
      .export_values();
}
