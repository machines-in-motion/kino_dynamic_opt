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

#include <momentumopt/dynopt/DynamicsState.hpp>

namespace momentumopt {

  // DynamicsState class function implementations
  DynamicsState::DynamicsState()
    : dtime_(0.0),
	  com_(Eigen::Vector3d::Zero()),
      amom_(Eigen::Vector3d::Zero()),
      lmom_(Eigen::Vector3d::Zero()),
      amomd_(Eigen::Vector3d::Zero()),
      lmomd_(Eigen::Vector3d::Zero())
  {
	for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++)
	{
      eefs_ids_[eff_id] = 0;
      cnts_ids_[eff_id] = -1;
      eefs_activation_[eff_id] = false;
	  eefs_frcs_[eff_id] = Eigen::Vector3d::Zero();
	  eefs_trqs_[eff_id] = Eigen::Vector3d::Zero();
	  eefs_cops_[eff_id] = Eigen::Vector3d::Zero();
	}
  }

  std::string DynamicsState::toString() const
  {
	std::stringstream text;
    text << "  dt      " << this->time() << "\n";
    text << "  com     " << this->centerOfMass().transpose() << "\n";
    text << "  lmom    " << this->linearMomentum().transpose() << "\n";
    text << "  amom    " << this->angularMomentum().transpose() << "\n";
    text << "  lmomd   " << this->linearMomentumRate().transpose() << "\n";
    text << "  amomd   " << this->angularMomentumRate().transpose() << "\n";

	for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++)
	{
	  text << "  eff_id " << eff_id << "\n";
      text << "    act  " << this->endeffectorActivation(eff_id) << "\n";
      text << "    cop  " << this->endeffectorCoP(eff_id).transpose() << "\n";
      text << "    frc  " << this->endeffectorForce(eff_id).transpose() << "\n";
      text << "    trq  " << this->endeffectorTorque(eff_id).transpose() << "\n";
	}
	return text.str();
  }

  void DynamicsState::fillInitialRobotState(const std::string cfg_file, const std::string robot_state)
  {
	try {
	  YAML::Node robot_cfg = YAML::LoadFile(cfg_file.c_str());
	  YAML::Node ini_robo_cfg = robot_cfg[robot_state.c_str()];
	  readParameter(ini_robo_cfg, "com", this->centerOfMass());
	  readParameter(ini_robo_cfg, "lmom", this->linearMomentum());
	  readParameter(ini_robo_cfg, "amom", this->angularMomentum());

	  for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
		Eigen::VectorXd eef_cfg = readParameter<Eigen::VectorXd>(ini_robo_cfg["eef_pose"], "eef_"+Problem::idToEndeffectorString(eff_id));
		this->endeffectorActivation(eff_id) = int(eef_cfg(0));
		readParameter(ini_robo_cfg["eef_ctrl"], "eef_frc_"+Problem::idToEndeffectorString(eff_id), this->endeffectorForce(eff_id));
	  }
	} catch (std::runtime_error& e) {
	  std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
	}
  }

  // DynamicsSequence class function implementations
  void DynamicsSequence::resize(int num_timesteps)
  {
    dynamics_sequence_.clear();
	for (int time_id=0; time_id<num_timesteps; time_id++)
		dynamics_sequence_.push_back(DynamicsState());
  }

  std::string DynamicsSequence::toString() const
  {
    std::stringstream text;
    for (int time_id=0; time_id<this->size(); time_id++) {
      text << "time " << time_id << "\n";
      text << dynamics_sequence_[time_id];
    }
	return text.str();
  }

}
