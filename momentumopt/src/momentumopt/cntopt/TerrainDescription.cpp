/**
 * @file TerrainDescription.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#include <momentumopt/cntopt/TerrainDescription.hpp>

namespace momentumopt {

  // TerrainRegion functions implementation
  void TerrainRegion::initialize(const Eigen::Matrix<double,Eigen::Dynamic,3>& ccwvertices, int terrain_id)
  {
    terrain_id_ = terrain_id;
    Eigen::Vector3d aux1, aux2, normal;

    // finding point and normal to the defined surface
    int num_rows = ccwvertices.rows();
    aux1 = ccwvertices.row(2) - ccwvertices.row(1);
    aux2 = ccwvertices.row(1) - ccwvertices.row(0);
    normal_ = aux1.cross(aux2);
    normal_ /= normal_.norm();
    point_.setZero();
    for (int row_id=0; row_id<num_rows; row_id++)
      point_ += ccwvertices.row(row_id);
    point_ /= num_rows;

    // build definition of half-spaces
    A_ = - ccwvertices;
    A_.block(0,0,num_rows-1,3) += ccwvertices.block(1,0,num_rows-1,3);
    A_.block(num_rows-1,0,1,3) += ccwvertices.block(0,0,1,3);
    for (int row_id=0; row_id<num_rows; row_id++) {
      aux1 = normal_.cross(A_.row(row_id));
      A_.row(row_id) = aux1 / aux1.norm();
    }

    Eigen::Matrix<double,Eigen::Dynamic,3> vmat;
    vmat = 0.5*ccwvertices;
    vmat.block(0,0,num_rows-1,3) += 0.5*ccwvertices.block(1,0,num_rows-1,3);
    vmat.block(num_rows-1,0,1,3) += 0.5*ccwvertices.block(0,0,1,3);
    b_.resize(num_rows);
    for (int row_id=0; row_id<num_rows; row_id++)
      b_(row_id) = vmat.row(row_id).dot(A_.row(row_id));

    // build rotation matrix
    aux1 = ccwvertices.row(0) - ccwvertices.row(1);
    aux2 = ccwvertices.row(1) - ccwvertices.row(2);
    normal = aux1.cross(aux2);
    rotation_.row(0) = aux1 / aux1.norm();
    rotation_.row(1) = aux2 / aux2.norm();
    rotation_.row(2) = normal / normal.norm();
  }

  std::string TerrainRegion::toString() const
  {
    std::stringstream text;
    Eigen::Quaternion<double> eig_q(this->terrainRotation());
    text << "    id          " << this->terrainId() << "\n";
    text << "    point       " << this->pointOnTerrain().transpose() << "\n";
    text << "    normal      " << this->normalToTerrain().transpose() << "\n";
    text << "    rotation  \n" << this->terrainRotation() << "\n";
    text << "    orientation " << eig_q.coeffs().transpose() << "\n";
    return text.str();
  }

  // TerrainDescription functions implementation
  void TerrainDescription::addTerrainRegion(const Eigen::Matrix<double,Eigen::Dynamic,3>& ccwvertices)
  {
    terrain_regions_.push_back(TerrainRegion());
    num_terrain_regions_ = terrain_regions_.size();
    terrain_regions_[num_terrain_regions_-1].initialize(ccwvertices, num_terrain_regions_-1);
  }

  std::string TerrainDescription::toString() const
  {
    std::stringstream text;
    for (int reg_id=0; reg_id<this->numRegions(); reg_id++) {
      text << "  ================================" << "\n";
      text << this->terrainRegion(reg_id);
    }
    return text.str();
  }

  void TerrainDescription::loadFromFile(const std::string cfg_file, const std::string terrain_description_name)
  {
    try {
      YAML::Node terrain_cfg = YAML::LoadFile(cfg_file.c_str());
      YAML::Node terrain_description = terrain_cfg[terrain_description_name.c_str()];

      std::vector<Eigen::Matrix<double, Eigen::Dynamic,3>> regions;
      YAML::ReadParameter(terrain_description, "regions", regions);

      for (unsigned int reg_id=0; reg_id<regions.size(); reg_id++)
        this->addTerrainRegion(regions[reg_id]);

    } catch (std::runtime_error& e) {
      std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }
  }

}
