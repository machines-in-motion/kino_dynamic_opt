/**
 * @file ContactState.hpp
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
   * Definition of contact type:
   *   FlatContact: forces subject to friction cone constrains of flat surface
   *   FullContact: forces free in any direction (e.g. hand grasp)
   */
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

      int& terrainId() { return terrain_id_; }
      int& optimizationId() { return optimization_id_; }
      ContactType& contactType() { return contact_type_; }
      double& contactActivationTime() { return time_ini_; }
      double& contactDeactivationTime() { return time_end_; }
      bool& selectedAsActive() { return selected_as_active_; }
      Eigen::Vector3d& contactPosition() { return position_; }
      Eigen::Quaternion<double>& contactOrientation() { return orientation_; }

      void contactActivationTime(const double& time_ini) { time_ini_ = time_ini; }
      void contactDeactivationTime(const double& time_end) { time_end_ = time_end; }
      void contactPosition(const Eigen::Vector3d& position) { position_ = position; }
      void contactOrientation(const Eigen::Quaternion<double>& orientation) { orientation_ = orientation; }
      void contactType(const ContactType& contact_type) { contact_type_ = contact_type; }

      const int& terrainId() const { return terrain_id_; }
      const int& optimizationId() const { return optimization_id_; }
      const ContactType& contactType() const { return contact_type_; }
      const double& contactActivationTime() const { return time_ini_; }
      const double& contactDeactivationTime() const { return time_end_; }
      const bool& selectedAsActive() const { return selected_as_active_; }
      const Eigen::Vector3d& contactPosition() const { return position_; }
      const Eigen::Quaternion<double>& contactOrientation() const { return orientation_; }

      std::string toString() const;
      friend std::ostream& operator<<(std::ostream &os, const ContactState& obj) { return os << obj.toString(); }

    private:
      friend class ContactSequence;
      ContactState(const Eigen::VectorXd& parameters, const int optimization_id);

    private:
      bool selected_as_active_;
      Eigen::Vector3d position_;
      ContactType contact_type_;
      double time_ini_, time_end_;
      int optimization_id_, terrain_id_;
      Eigen::Quaternion<double> orientation_;
  };

  /**
   * This class is a container for a sequence of contact states,
   * for all end-effectors.
   */
  class ContactSequence
  {
    public:
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
      ContactSequence(){}
      ~ContactSequence(){}

      const int numOptimizationContacts() { return num_optimization_contacts_; }
      const int numEndeffectorContacts(int eff_id) const { return endeffector_contacts_[eff_id].size(); }
      void loadFromFile(const std::string cfg_file, const std::string contact_plan_name = "contact_plan");
      std::vector<ContactState>& endeffectorContacts(int eff_id) { return endeffector_contacts_[eff_id]; }
      const std::vector<ContactState>& endeffectorContacts(int eff_id) const { return endeffector_contacts_[eff_id]; }
      void endeffectorContacts(int eff_id, const std::vector<ContactState>& endeffector_contacts) { endeffector_contacts_[eff_id] = endeffector_contacts; }
      const long endeffectorNum() { return endeffector_contacts_.size(); }

      std::string toString() const;
      friend std::ostream& operator<<(std::ostream &os, const ContactSequence& obj) { return os << obj.toString(); }

    private:
      int num_optimization_contacts_;
      std::array<std::vector<ContactState>, Problem::n_endeffs_> endeffector_contacts_;
  };

  /**
   * This class defines a viapoint state including position and orientation
   */
  class ViapointState
  {
    public:
      ViapointState();
      ~ViapointState(){}

      int& viapointId() { return viapoint_id_; }
      Eigen::Vector3d& viapointPosition() { return position_; }
      Eigen::Quaternion<double>& viapointOrientation() { return orientation_; }

      const int& viapointId() const { return viapoint_id_; }
      const int& optimizationId() { return optimization_id_; }
      const Eigen::Vector3d& viapointPosition() const { return position_; }
      const Eigen::Quaternion<double>& viapointOrientation() const { return orientation_; }

      std::string toString() const;
      friend std::ostream& operator<<(std::ostream &os, const ViapointState& obj) { return os << obj.toString(); }

    private:
      friend class ViapointSequence;
      ViapointState(const Eigen::VectorXd& parameters, const int optimization_id);

    private:
      Eigen::Vector3d position_;
      int optimization_id_, viapoint_id_;
      Eigen::Quaternion<double> orientation_;
  };

  /**
   * This class is a container for a sequence of viapoint states for all end-effectors.
   */
  class ViapointSequence
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
      ViapointSequence(){}
      ~ViapointSequence(){}

      const int& numOptimizationViapoints() const { return num_optimization_viapoints_; }
      const int  numEndeffectorViapoints(int eff_id) const { return endeffector_viapoints_[eff_id].size(); }
      void loadFromFile(const std::string cfg_file, const std::string contact_plan_name = "contact_plan");
      std::vector<ViapointState>& endeffectorViapoints(int eff_id) { return endeffector_viapoints_[eff_id]; }
      const std::vector<ViapointState>& endeffectorViapoints(int eff_id) const { return endeffector_viapoints_[eff_id]; }

      std::string toString() const;
      friend std::ostream& operator<<(std::ostream &os, const ViapointSequence& obj) { return os << obj.toString(); }

    private:
      int num_optimization_viapoints_;
      std::array<std::vector<ViapointState>, Problem::n_endeffs_> endeffector_viapoints_;
  };


}
