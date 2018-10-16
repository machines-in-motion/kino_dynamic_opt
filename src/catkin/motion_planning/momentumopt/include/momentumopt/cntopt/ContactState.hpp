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

  enum class ContactType { FreeContact = 0, FlatContact = 1, FullContact = 2 };
  ContactType idToContactType(int cnt_type_id);

  /**
   * This class is a container for all variables required to define a
   * contact state: position and orientation of the contact, time of
   * activation and de-activation of the contact, and a ContactType
   * variable indicating whether, forces are subject to a friction cone
   * (ContactType::FlatContact) or not (ContactType::FullContact) such as in
   * the case of a hand grasping a bar.
   */
  class ContactState
  {
    public:
      ContactState();
      ~ContactState(){}

      ContactType& contactType() { return contact_type_; }
      double& contactActivationTime() { return time_ini_; }
      double& contactDeactivationTime() { return time_end_; }
      Eigen::Vector3d& contactPosition() { return position_; }
      Eigen::Quaternion<double>& contactOrientation() { return orientation_; }

      const ContactType& contactType() const { return contact_type_; }
      const double& contactActivationTime() const { return time_ini_; }
      const double& contactDeactivationTime() const { return time_end_; }
      const Eigen::Vector3d& contactPosition() const { return position_; }
      const Eigen::Quaternion<double>& contactOrientation() const { return orientation_; }

      std::string toString() const;
      friend std::ostream& operator<<(std::ostream &os, const ContactState& obj) { return os << obj.toString(); }

    private:
      Eigen::Vector3d position_;
      ContactType contact_type_;
      double time_ini_, time_end_;
      Eigen::Quaternion<double> orientation_;
  };

  /**
   * This class is a container for a sequence of contact states,
   * for all end-effectors.
   */
  class ContactSequence
  {
    public:
      ContactSequence(){}
      ~ContactSequence(){}

      std::vector<ContactState>& endeffectorContacts(int eff_id) { return endeffector_contacts_[eff_id]; }
      const std::vector<ContactState>& endeffectorContacts(int eff_id) const { return endeffector_contacts_[eff_id]; }

      std::string toString() const;
      friend std::ostream& operator<<(std::ostream &os, const ContactSequence& obj) { return os << obj.toString(); }

    private:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      std::array<std::vector<ContactState>, Problem::n_endeffs_> endeffector_contacts_;
  };

}
