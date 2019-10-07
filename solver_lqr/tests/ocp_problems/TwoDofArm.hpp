/**
 * @file TwoDofArm.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-07
 */

#pragma once

#include <yaml_cpp_catkin/yaml_eigen.h>
#include <solver_lqr/OcpDescription.hpp>

namespace ocp_test_problems
{
  class TwoDofArm : public solverlqr::OcpBase
  {
    public:
	  TwoDofArm() : OcpBase() {}
      ~TwoDofArm() {}

    public:
      Eigen::Matrix2d MassM(const solverlqr::StateBase& state)
      {
    	Eigen::Matrix2d mass;
    	double q1=state.stateVector()[0], q2=state.stateVector()[1];

    	double Ia = 0.16*sin(q1+q2) + 0.3*sin(q1);
    	double Ib = 0.16*cos(q1+q2) + 0.3*cos(q1);
    	double Ic = 0.16*(Ia*sin(q1+q2)+Ib*cos(q1+q2)) + 0.045;
        mass << Ia*Ia + Ib*Ib + 0.08694,   Ic,
				Ic,                        0.0706;
        return mass;
      }

      Eigen::Vector2d CoriolisV(const solverlqr::StateBase& state)
      {
        Eigen::Vector2d coriolis;
        double q1=state.stateVector()[0], q2=state.stateVector()[1],
               dq1=state.stateVector()[2], dq2=state.stateVector()[3];

        double c12 = cos(q1+q2);
        double s12 = sin(q1+q2);

        double Ca = 0.16*s12 + 0.3*sin(q1);
        double Cb = 0.16*c12 + 0.3*cos(q1);
        double Cc = 0.16*c12*dq2 + Cb*dq1;
        double Cd = 0.16*s12*dq2 + Ca *dq1;
        double Ce = 0.16*(c12*dq2 + c12*dq1);
        double Cf = 0.16*(s12*dq2 + s12*dq1);
        coriolis << dq1*(Ca*Cc-Cb*Cd) + dq2*(Ca*Ce-Cb*Cf),
        		    0.16*(dq1*(s12*Cc-c12*Cd) + dq2*(s12*Ce-c12*Cf));
        return coriolis;
      }

      Eigen::Vector2d GravityV(const solverlqr::StateBase& state)
      {
    	Eigen::Vector2d gravity;
    	gravity.setZero();
    	return gravity;
      }

      Eigen::Matrix2d SelectionM(const solverlqr::StateBase& state)
      {
    	Eigen::Matrix2d selection;
    	selection.setIdentity();
    	return selection;
      }

      Eigen::Matrix2d JacobianM(const solverlqr::StateBase& state)
      {
    	Eigen::Matrix2d jacobian;
    	double q1=state.stateVector()[0], q2=state.stateVector()[1];
        double s1=sin(q1), c1=cos(q1), s12=sin(q1+q2), c12=cos(q1+q2);
    	jacobian << -0.3*s1-0.33*s12,   -0.33*s12,
    	             0.3*c1+0.33*c12,    0.33*c12;
		return jacobian;
      }

      Eigen::Vector2d ForwKin(const solverlqr::StateBase& state)
      {
        Eigen::Vector2d cart_pos;
        double q1=state.stateVector()[0], q2=state.stateVector()[1];
        cart_pos << 0.3*cos(q1) + 0.33*cos(q1+q2),
                    0.3*sin(q1) + 0.33*sin(q1+q2);
        return cart_pos;
      }

      void configure(const YAML::Node& user_parameters)
      {
        try
        {
          xgoal_ = user_parameters["xgoal"].as<Eigen::Vector4d>();
          Qx_ = user_parameters["Qx"].as<Eigen::Vector4d>().asDiagonal();
          Qu_ = user_parameters["Qu"].as<Eigen::Vector2d>().asDiagonal();
          Qf_ = user_parameters["Qf"].as<Eigen::Vector4d>().asDiagonal();
          process_filter_ = user_parameters["process_filter"].as<Eigen::Vector4d>().asDiagonal();
		  measurement_filter_ = user_parameters["measurement_filter"].as<Eigen::Vector4d>().asDiagonal();

          nviapoints_ = user_parameters["nviapoints"].as<int>();
          if (nviapoints_>0) {
            tvia_ = user_parameters["tvia"].as<Eigen::VectorXd>();
            for (int via=0; via<nviapoints_; via++) {
              std::stringstream via_id; via_id << "via" << via;
              pvia_.push_back(user_parameters[via_id.str()].as<Eigen::VectorXd>());
            }
          }
    	}
    	catch (...)
    	{
    	  throw std::runtime_error("Err reading TwoDofArm params (Fnc configure). \n");
    	}
      }

      double objective(const solverlqr::StateBase& state, const solverlqr::ControlBase& control, int time_id, bool is_final_timestep)
      {
        Eigen::Vector4d cart_state;
        cart_state.head(this->xdim()/2) = this->ForwKin(state);
        cart_state.tail(this->xdim()/2) = this->JacobianM(state)*state.stateVector().tail(this->xdim()/2);

      	double objective = this->dt() * control.feedforward().dot( Qu_*control.feedforward() );
      	for (int id=0; id<nviapoints_; id++) {
      	  if (time_id == std::floor(tvia_[id]/this->dt()))
      		objective += (cart_state - pvia_[id]).dot( Qx_*(cart_state - pvia_[id]) );
      	}
        if (is_final_timestep) { objective = (cart_state - xgoal_).dot( Qf_*(cart_state - xgoal_) ); }
        return objective;
      }

      solverlqr::StateBase dynamics(const solverlqr::StateBase& state, const solverlqr::ControlBase& control, int time_id)
      {
        Eigen::LLT<Eigen::Matrix2d> mass_inv;
        mass_inv.compute(this->MassM(state));

        solverlqr::StateBase dstate_dt(this->xdim());
        dstate_dt.stateVector().head(this->xdim()/2) = state.stateVector().tail(this->xdim()/2);
        dstate_dt.stateVector().tail(this->xdim()/2) = mass_inv.solve(this->SelectionM(state)*control.feedforward() - this->GravityV(state) - this->CoriolisV(state));
        return this->dt()*dstate_dt;
      }

      Eigen::MatrixXd processNoiseFilter(int time_id) const
      {
    	    return process_filter_;
      }

      Eigen::MatrixXd measurementNoiseFilter(int time_id) const
      {
        return measurement_filter_;
      }

    private:
      int nviapoints_;
      Eigen::Matrix2d Qu_;
      Eigen::VectorXd tvia_;
      Eigen::Vector4d xgoal_;
      std::vector<Eigen::Vector4d> pvia_;
      Eigen::Matrix4d Qx_, Qf_, process_filter_, measurement_filter_;

  };
}
