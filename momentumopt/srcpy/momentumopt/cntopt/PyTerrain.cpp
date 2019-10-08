/**
 * @file PyTerrain.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>

#include <momentumopt/cntopt/TerrainDescription.hpp>

namespace py = pybind11;
using namespace momentumopt;

void init_terrain(py::module &m)
{
  py::class_<TerrainDescription>(m, "TerrainDescription")
    .def(py::init<>())
    .def("addTerrainRegion", &TerrainDescription::addTerrainRegion)
	.def("loadFromFile", &TerrainDescription::loadFromFile, py::arg("cfg_file"), py::arg("terrain_description_name") = "terrain_description")

    .def("__repr__", [](const TerrainDescription &terrain) { return terrain.toString(); } );
}
