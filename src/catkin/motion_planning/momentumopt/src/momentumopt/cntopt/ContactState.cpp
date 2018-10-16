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
      position_(Eigen::Vector3d::Zero()),
      contact_type_(ContactType::FreeContact),
      orientation_(Eigen::Quaternion<double>::Identity())
  {
  }

  std::string ContactState::toString() const
  {
    std::stringstream text;
    text << "    time         " << time_ini_ << " - " << time_end_ << "\n";
    text << "    contact type " << static_cast<int>(contact_type_) << "\n";
    text << "    position     " << position_.transpose() << "\n";
    text << "    orientation  " << orientation_.coeffs().transpose() << "\n";
    return text.str();
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
}
