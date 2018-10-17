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

#include <momentumopt/cntopt/ContactState.hpp>

namespace momentumopt {

  // ContactState functions implementation
  ContactType idToContactType(int cnt_type_id)
  {
    ContactType cnt_type = ContactType::FreeContact;
    switch (cnt_type_id) {
      case 0: { cnt_type = ContactType::FreeContact; break; }
      case 1: { cnt_type = ContactType::FlatContact; break; }
      case 2: { cnt_type = ContactType::FullContact; break; }
    }
    return cnt_type;
  }

  ContactState::ContactState()
    : time_ini_(0.),
      time_end_(0.),
      contact_id_(-1),
      terrain_id_(-1),
      selected_as_active_(false),
      position_(Eigen::Vector3d::Zero()),
      contact_type_(ContactType::FreeContact),
      orientation_(Eigen::Quaternion<double>::Identity())
  {
  }

  ContactState::ContactState(const Eigen::VectorXd& parameters, const int contact_id)
  {
    contact_id_ = contact_id;
    time_ini_ = parameters(0);
    time_end_ = parameters(1);
    selected_as_active_ = false;
    terrain_id_ = parameters(10);
    position_ = parameters.segment<3>(2);
    contact_type_ = idToContactType(parameters(9));
    orientation_ = Eigen::Quaternion<double>(parameters(5), parameters(6), parameters(7), parameters(8));
  }

  std::string ContactState::toString() const
  {
    std::stringstream text;
    text << "    is active    " << selected_as_active_ << "\n";
    text << "    contact id   " << contact_id_ << "\n";
    text << "    terrain id   " << terrain_id_ << "\n";
    text << "    time         " << time_ini_ << " - " << time_end_ << "\n";
    text << "    contact type " << static_cast<int>(contact_type_) << "\n";
    text << "    position     " << position_.transpose() << "\n";
    text << "    orientation  " << orientation_.coeffs().transpose() << "\n";
    return text.str();
  }

  // ContactSequence functions implementation
  void ContactSequence::loadFromFile(const std::string cfg_file, const std::string contact_plan_name)
  {
    try {
      YAML::Node contact_cfg = YAML::LoadFile(cfg_file.c_str());
      YAML::Node contact_plan = contact_cfg[contact_plan_name.c_str()];

      num_contacts_ = 0;
      std::vector<Eigen::VectorXd> contacts;
      for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
        contacts.clear();
        readParameter(contact_plan, ("effcnt_" + Problem::idToEndeffectorString(eff_id)).c_str(), contacts);

        this->endeffectorContacts(eff_id).clear();
        for (int cnt_id=0; cnt_id<contacts.size(); cnt_id++)
          this->endeffectorContacts(eff_id).push_back(ContactState(contacts[cnt_id], num_contacts_++));
      }
    } catch (std::runtime_error& e) {
      std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }
  }

  std::string ContactSequence::toString() const
  {
	std::stringstream text;
	for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
	  text << "eff_id " << eff_id << "\n";
      for (int cnt_id=0; cnt_id<endeffector_contacts_[eff_id].size(); cnt_id++) {
        text << "  cnt_id " << cnt_id << "\n";
        text << endeffector_contacts_[eff_id][cnt_id];
      }
	}
	return text.str();
  }

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
    for (int row_id=0; row_id<num_rows; row_id++) { point_ += ccwvertices.row(row_id); }
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
      readParameter(terrain_description, "regions", regions);

      for (int reg_id=0; reg_id<regions.size(); reg_id++)
        this->addTerrainRegion(regions[reg_id]);

    } catch (std::runtime_error& e) {
      std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }
  }

}
