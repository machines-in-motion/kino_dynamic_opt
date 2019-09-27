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
    : selected_as_active_(false),
      position_(Eigen::Vector3d::Zero()),
      contact_type_(ContactType::FreeContact),
      time_ini_(0.),
      time_end_(0.),
      optimization_id_(-1),
      terrain_id_(-1),
      orientation_(Eigen::Quaternion<double>::Identity())
  {
  }

  ContactState::ContactState(const Eigen::VectorXd& parameters, const int optimization_id)
  {
    time_ini_ = parameters(0);
    time_end_ = parameters(1);
    selected_as_active_ = false;
    terrain_id_ = parameters(10);
    optimization_id_ = optimization_id;
    position_ = parameters.segment<3>(2);
    contact_type_ = idToContactType(parameters(9));
    orientation_ = Eigen::Quaternion<double>(parameters(5), parameters(6), parameters(7), parameters(8));
  }

  std::string ContactState::toString() const
  {
    std::stringstream text;
    text << "    is active        " << selected_as_active_ << "\n";
    text << "    optimization id  " << optimization_id_ << "\n";
    text << "    terrain id       " << terrain_id_ << "\n";
    text << "    time             " << time_ini_ << " - " << time_end_ << "\n";
    text << "    contact type     " << static_cast<int>(contact_type_) << "\n";
    text << "    position         " << position_.transpose() << "\n";
    text << "    orientation      " << orientation_.coeffs().transpose() << "\n";
    return text.str();
  }

  // ContactSequence functions implementation
  void ContactSequence::loadFromFile(const std::string cfg_file, const std::string contact_plan_name)
  {
    try {
      YAML::Node contact_cfg = YAML::LoadFile(cfg_file.c_str());
      YAML::Node contact_plan = contact_cfg[contact_plan_name.c_str()];

      num_optimization_contacts_ = 0;
      std::vector<Eigen::VectorXd> contacts;
      for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
        contacts.clear();
        readParameter(contact_plan, ("effcnt_" + Problem::idToEndeffectorString(eff_id)).c_str(), contacts);

        this->endeffectorContacts(eff_id).clear();
        for (unsigned int cnt_id=0; cnt_id<contacts.size(); cnt_id++)
          this->endeffectorContacts(eff_id).push_back(ContactState(contacts[cnt_id], num_optimization_contacts_++));
      }
    } catch (std::runtime_error& e) {
      std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }
  }

  std::string ContactSequence::toString() const
  {
    std::stringstream text;
    for (unsigned int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
      text << "eff_id " << eff_id << "\n";
      for (unsigned int cnt_id=0; cnt_id<endeffector_contacts_[eff_id].size(); cnt_id++) {
        text << "  cnt_id " << cnt_id << "\n";
        text << endeffector_contacts_[eff_id][cnt_id];
      }
	}
	return text.str();
  }

  // ViapointState functions implementation
  ViapointState::ViapointState()
    : position_(Eigen::Vector3d::Zero()),
      optimization_id_(-1),
      viapoint_id_(-2),      
      orientation_(Eigen::Quaternion<double>::Identity())
  {
  }

  ViapointState::ViapointState(const Eigen::VectorXd& parameters, const int optimization_id)
  {
    viapoint_id_ = parameters(0);
    optimization_id_ = optimization_id;
    position_ = parameters.segment<3>(1);
    orientation_ = Eigen::Quaternion<double>(parameters(4), parameters(5), parameters(6), parameters(7));
  }

  std::string ViapointState::toString() const
  {
    std::stringstream text;
    text << "    position     " << position_.transpose() << "\n";
    text << "    orientation  " << orientation_.coeffs().transpose() << "\n";
    return text.str();
  }

  // ViapointSequence functions implementation
  void ViapointSequence::loadFromFile(const std::string cfg_file, const std::string contact_plan_name)
  {
    try {
      YAML::Node contact_cfg = YAML::LoadFile(cfg_file.c_str());
      YAML::Node contact_plan = contact_cfg[contact_plan_name.c_str()];

      num_optimization_viapoints_ = 0;
      std::vector<Eigen::VectorXd> viapoints;
      for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
        viapoints.clear();
        readParameter(contact_plan, ("effvia_" + Problem::idToEndeffectorString(eff_id)).c_str(), viapoints);

        this->endeffectorViapoints(eff_id).clear();
        for (unsigned int via_id=0; via_id<viapoints.size(); via_id++)
          this->endeffectorViapoints(eff_id).push_back(ViapointState(viapoints[via_id], num_optimization_viapoints_++));
      }
    } catch (std::runtime_error& e) {
      std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }
  }

  std::string ViapointSequence::toString() const
  {
	std::stringstream text;
	for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
	  text << "eff_id " << eff_id << "\n";
      for (int via_id=0; via_id<(int)endeffector_viapoints_[eff_id].size(); via_id++) {
        text << "  via_id " << via_id << "\n";
        text << endeffector_viapoints_[eff_id][via_id];
      }
	}
	return text.str();
  }

}
