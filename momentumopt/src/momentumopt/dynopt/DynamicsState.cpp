/**
 * @file DynamicsState.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#include <momentumopt/dynopt/DynamicsState.hpp>

namespace momentumopt {

  // DynamicsState class function implementations
  DynamicsState::DynamicsState()
    : dtime_(0.1),
      com_(Eigen::Vector3d::Zero()),
      amom_(Eigen::Vector3d::Zero()),
      lmom_(Eigen::Vector3d::Zero()),
      amomd_(Eigen::Vector3d::Zero()),
      lmomd_(Eigen::Vector3d::Zero())
  {
    eff_positions_.clear();
    eff_velocities_.clear();
    eff_orientations_.clear();
    eff_accelerations_.clear();


	for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++)
	{
      eff_ids_[eff_id] = 0;
      cnt_ids_[eff_id] = -1;
      eff_activations_[eff_id] = false;

      eff_cops_.push_back(Eigen::Vector3d::Zero());
	  eff_forces_.push_back(Eigen::Vector3d::Zero());
	  eff_torques_.push_back(Eigen::Vector3d::Zero());
	  eefs_trqs_contact_point_.push_back(Eigen::Vector3d::Zero());

	  eff_positions_.push_back(Eigen::Vector3d::Zero());
	  eff_velocities_.push_back(Eigen::Vector3d::Zero());
	  eff_accelerations_.push_back(Eigen::Vector3d::Zero());
	  eff_orientations_.push_back(Eigen::Quaternion<double>::Identity());
	}
  }

  std::string DynamicsState::toString() const
  {
	std::stringstream text;
    //text << "  dt      " << this->time() << "\n";
    text << "  com     " << this->centerOfMass().transpose() << "\n";
    //text << "  lmom    " << this->linearMomentum().transpose() << "\n";
    //text << "  amom    " << this->angularMomentum().transpose() << "\n";
    //text << "  lmomd   " << this->linearMomentumRate().transpose() << "\n";
    //text << "  amomd   " << this->angularMomentumRate().transpose() << "\n";

	for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
	  text << "  eff_id " << eff_id << "\n";
      //text << "    act  " << this->endeffectorActivation(eff_id) << "\n";
      //text << "    cop  " << this->endeffectorCoP(eff_id).transpose() << "\n";
      //text << "    frc  " << this->endeffectorForce(eff_id).transpose() << "\n";
      //text << "    trq  " << this->endeffectorTorque(eff_id).transpose() << "\n";
      text << "    pos  " << this->endeffectorPosition(eff_id).transpose() << "\n";
      //text << "    ori  " << this->endeffectorOrientation(eff_id).coeffs().transpose() << "\n";
	}
	return text.str();
  }

  void DynamicsState::fillInitialRobotState(const std::string cfg_file, const std::string robot_state)
  {
	try {
	  YAML::Node robot_cfg = YAML::LoadFile(cfg_file.c_str());
	  YAML::Node ini_robo_cfg = robot_cfg[robot_state.c_str()];
	  YAML::ReadParameter(ini_robo_cfg, "com", this->centerOfMass());
	  YAML::ReadParameter(ini_robo_cfg, "lmom", this->linearMomentum());
	  YAML::ReadParameter(ini_robo_cfg, "amom", this->angularMomentum());

	  for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
		Eigen::VectorXd eef_cfg = YAML::ReadParameter<Eigen::VectorXd>(ini_robo_cfg["eef_pose"], "eef_"+Problem::idToEndeffectorString(eff_id));
		this->endeffectorActivation(eff_id) = int(eef_cfg(0));
		this->endeffectorPosition(eff_id) = eef_cfg.segment<3>(1);
		this->endeffectorOrientation(eff_id) = Eigen::Quaternion<double>(eef_cfg[4],eef_cfg[5],eef_cfg[6],eef_cfg[7]);
		YAML::ReadParameter(ini_robo_cfg["eef_ctrl"], "eef_frc_"+Problem::idToEndeffectorString(eff_id), this->endeffectorForce(eff_id));
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
