/**
 * @file TerrainDescription.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
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
      int numRegions() const { return terrain_regions_.size(); }
      TerrainRegion& terrainRegion(int terrain_id) { return terrain_regions_[terrain_id]; }
      const TerrainRegion& terrainRegion(int terrain_id) const { return terrain_regions_[terrain_id]; }

      std::string toString() const;
      friend std::ostream& operator<<(std::ostream &os, const TerrainDescription& obj) { return os << obj.toString(); }

    private:
      int num_terrain_regions_;
      std::vector<TerrainRegion> terrain_regions_;
  };

}
