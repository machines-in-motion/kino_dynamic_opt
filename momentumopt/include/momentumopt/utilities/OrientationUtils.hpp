/**
 * @file OrientationUtils.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#pragma once

#include <Eigen/Geometry>

namespace momentumopt {

  /** computes the error between two quaternions by rotating the desired orientation
   *  back by the current orientation, and taking the logarithm of the result.
   */
  Eigen::Vector3d orientationError(const Eigen::Quaternion<double>& desired_orientation,
                                   const Eigen::Quaternion<double>& current_orientation);

  /** computation of the required angular velocity to move from a current orientation
   *  to a desired orientation in a given time interval. Angular velocity can be returned
   *  in body or world coordinates.
   */
  Eigen::Vector3d requiredAngularVelocity(const Eigen::Quaternion<double>& desired_orientation,
                                          const Eigen::Quaternion<double>& current_orientation,
                                          const double& time_step, bool in_body_coordinates=false);

  /** integration of a quaternion given an angular velocity given either in
   *  body or world coordinates and a time interval for integration
   */
  Eigen::Quaternion<double> integrateAngularVelocity(const Eigen::Quaternion<double>& current_orientation,
                                                     const Eigen::Vector3d& angular_velocity,
                                                     const double& time_step, bool in_body_coordinates=false);

}
