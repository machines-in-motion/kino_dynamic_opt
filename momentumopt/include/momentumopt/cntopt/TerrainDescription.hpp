/*
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

#pragma once

#include <array>
#include <vector>
#include <memory>
#include <iostream>
#include <Eigen/Eigen>

#include <momentumopt/setting/Definitions.hpp>

namespace momentumopt {

  /**
   * This class is a container for all variables required to define a terrain region
   */
  class TerrainRegion
  {
    public:
      TerrainRegion() : terrain_id_(-1) {}
      ~TerrainRegion(){}

      /*! Getter and setter methods */
      int& terrainId() { return terrain_id_; }
      Eigen::Vector3d& pointOnTerrain() { return point_; }
      Eigen::Vector3d& normalToTerrain() { return normal_; }
      Eigen::Matrix3d& terrainRotation() { return rotation_; }
      Eigen::VectorXd& terrainDescriptionVector() { return b_; }
      Eigen::Matrix<double,Eigen::Dynamic,3>& terrainDescriptionMatrix() { return A_; }

      const int& terrainId() const { return terrain_id_; }
      const Eigen::Vector3d& pointOnTerrain() const { return point_; }
      const Eigen::Vector3d& normalToTerrain() const { return normal_; }
      const Eigen::Matrix3d& terrainRotation() const { return rotation_; }
      const Eigen::VectorXd& terrainDescriptionVector() const { return b_; }
      const Eigen::Matrix<double,Eigen::Dynamic,3>& terrainDescriptionMatrix() const { return A_; }
      const Eigen::Quaternion<double> terrainOrientation() const { return Eigen::Quaternion<double>(rotation_); }

      std::string toString() const;
      friend std::ostream& operator<<(std::ostream &os, const TerrainRegion& obj) { return os << obj.toString(); }

    private:
      friend class TerrainDescription;

      /* initialization function only available to terrain description */
      void initialize(const Eigen::Matrix<double,Eigen::Dynamic,3>& ccwvertices, int terrain_id);

    private:
      int terrain_id_;
      Eigen::VectorXd b_;
      Eigen::Matrix3d rotation_;
      Eigen::Vector3d point_, normal_;
      Eigen::Matrix<double,Eigen::Dynamic,3> A_;
  };

  /**
   * This class is a container for a terrain description in terms of terrain surfaces
   */
  class TerrainDescription
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
      TerrainDescription() : num_terrain_regions_(-1) {}
      ~TerrainDescription(){}

      void addTerrainRegion(const Eigen::Matrix<double,Eigen::Dynamic,3>& ccwvertices);
      void loadFromFile(const std::string cfg_file, const std::string terrain_description_name = "terrain_description");

      /*! Getter and setter methods */
      const int numRegions() const { return terrain_regions_.size(); }
      TerrainRegion& terrainRegion(int terrain_id) { return terrain_regions_[terrain_id]; }
      const TerrainRegion& terrainRegion(int terrain_id) const { return terrain_regions_[terrain_id]; }

      std::string toString() const;
      friend std::ostream& operator<<(std::ostream &os, const TerrainDescription& obj) { return os << obj.toString(); }

    private:
      int num_terrain_regions_;
      std::vector<TerrainRegion> terrain_regions_;
  };

}
