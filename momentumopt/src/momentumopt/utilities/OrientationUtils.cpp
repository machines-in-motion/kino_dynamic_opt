/**
 * @file OrientationUtils.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#include <momentumopt/utilities/OrientationUtils.hpp>

namespace momentumopt {

  // skew matrix: vector to skew symmetric matrix
  Eigen::Matrix3d skewMatrix(const Eigen::Vector3d& vec)
  {
    Eigen::Matrix3d mat;
    mat <<      0.0, -vec.z(),  vec.y(),
            vec.z(),      0.0, -vec.x(),
           -vec.y(),  vec.x(),      0.0;
    return mat;
  }

  // logarithmic map: quaternion -> rotation vector
  Eigen::Vector3d logarithmicMap(const Eigen::Quaternion<double>& quaternion)
  {
    Eigen::Quaternion<double> normalized_quaternion = quaternion;
    normalized_quaternion.normalize();
    if (normalized_quaternion.w() < 0.0) {
      normalized_quaternion.w() *= -1.0;
      normalized_quaternion.vec() *= -1.0;
    }

    if (std::abs(1.0-normalized_quaternion.w()) < 0.01) {
      return 2.0*normalized_quaternion.vec();
    } else {
      double norm = 2.0*std::acos(normalized_quaternion.w());
      return norm/std::sin(0.5*norm) * normalized_quaternion.vec();
    }
  }

  // exponential map: rotation vector -> quaternion
  Eigen::Quaternion<double> exponentialMap(const Eigen::Vector3d& rotation_vector)
  {
    Eigen::Quaternion<double> quaternion;
    if (rotation_vector.norm() < 0.01) {
      quaternion = Eigen::Quaternion<double>(1.0, 0.5*rotation_vector.x(), 0.5*rotation_vector.y(), 0.5*rotation_vector.z());
    } else {
      double norm = std::cos(rotation_vector.norm()/2.0);
      quaternion.w() = norm;
      quaternion.vec() = rotation_vector/rotation_vector.norm() * std::sin(0.5*rotation_vector.norm());
    }
    quaternion.normalize();
    if (quaternion.w() < 0.0) {
      quaternion.w() *= -1.0;
      quaternion.vec() *= -1.0;
    }
    return quaternion;
  }

  // quaternion matrix
  Eigen::Matrix4d toQuaternionMatrix(const Eigen::Quaternion<double>& quaternion, bool is_conjugate)
  {
    Eigen::Matrix4d q_matrix = quaternion.w() * Eigen::Matrix4d::Identity();
    q_matrix.bottomLeftCorner<3,1>() = quaternion.vec();
    q_matrix.topRightCorner<1,3>() = -quaternion.vec();
    if (!is_conjugate) { q_matrix.bottomRightCorner<3,3>() += skewMatrix(quaternion.vec()); }
    else               { q_matrix.bottomRightCorner<3,3>() -= skewMatrix(quaternion.vec()); }
    return q_matrix;
  }

  // jacobian of quaternion with respect to rotation vector
  Eigen::Matrix<double,4,3> jacobianQuaternionWrtRotationVector(const Eigen::Vector3d& rotation_vector)
  {
    Eigen::Matrix<double,4,3> jacobian = Eigen::Matrix<double,4,3>::Zero();
    if (rotation_vector.norm() < 0.01) {
      jacobian.topLeftCorner<1,3>() = -0.25*rotation_vector;
      jacobian.bottomLeftCorner<3,3>() = 0.5*Eigen::Matrix3d::Identity();
    } else {
      double factor1 = 0.5*std::sin(0.5*rotation_vector.norm())/rotation_vector.norm();
      double factor2 = ( std::cos(0.5*rotation_vector.norm())*rotation_vector.norm() - 2.0*std::sin(0.5*rotation_vector.norm()) )
    		               / ( 2.0*std::pow(rotation_vector.norm(), 3.0) );
      jacobian.topLeftCorner<1,3>() = -factor1*rotation_vector.transpose();
      jacobian.bottomLeftCorner<3,3>() = 2.0*factor1*Eigen::Matrix3d::Identity() + factor2*(rotation_vector*rotation_vector.transpose());
    }
    return jacobian;
  }

  // jacobian of angular velocity with respect rotation vector rates
  Eigen::Matrix3d jacobianAngularVelocityWrtRotationVector(const Eigen::Vector3d& rotation_vector)
  {
    Eigen::Matrix<double,3,4> jacobianAngularVelocityWrtQuaternion = toQuaternionMatrix(exponentialMap(rotation_vector), false).bottomRows<3>();
    jacobianAngularVelocityWrtQuaternion.bottomLeftCorner<3,1>() *= -1.0;
	return jacobianAngularVelocityWrtQuaternion * jacobianQuaternionWrtRotationVector(rotation_vector);
  }

  // to angular velocity: rotation vector rates -> angular velocity
  Eigen::Vector3d toAngularVelocity(const Eigen::Vector3d& rotation_vector, const Eigen::Vector3d& rotation_vector_rates, bool in_body_coordinates)
  {
    if (!in_body_coordinates) { return 2.0 * jacobianAngularVelocityWrtRotationVector(rotation_vector) * rotation_vector_rates; }
    else                      { return 2.0 * jacobianAngularVelocityWrtRotationVector(rotation_vector).transpose() * rotation_vector_rates; }
  }

  // from angular velocity: angular velocity -> rotation vector rates
  Eigen::Vector3d fromAngularVelocity(const Eigen::Vector3d& rotation_vector, const Eigen::Vector3d& angular_velocity, bool in_body_coordinates)
  {
    if (!in_body_coordinates) { return 0.5 * jacobianAngularVelocityWrtRotationVector(rotation_vector).inverse() * angular_velocity; }
    else                      { return 0.5 * jacobianAngularVelocityWrtRotationVector(rotation_vector).inverse().transpose() * angular_velocity; }
  }

  // required angular velocity to rotate from current to desired orientation during a time interval
  Eigen::Vector3d requiredAngularVelocity(const Eigen::Quaternion<double>& desired_orientation,
                                          const Eigen::Quaternion<double>& current_orientation,
                                          const double& time_step, bool in_body_coordinates)
  {
    Eigen::Quaternion<double> q_desired = desired_orientation;    q_desired.normalize();
    Eigen::Quaternion<double> q_current = current_orientation;    q_current.normalize();
    if (q_current.dot(q_desired) < 0.0) { q_current.w() *= -1.0; q_current.vec() *= -1.0; }

    Eigen::Quaternion<double> delta_quaternion = q_desired*q_current.inverse();
    Eigen::Vector3d delta_rotation = logarithmicMap(delta_quaternion);
    Eigen::Vector3d current_rotation = logarithmicMap(q_current);

    return toAngularVelocity(current_rotation, delta_rotation/time_step, in_body_coordinates);
  }

  // integration of current orientation given an angular velocity and a time interval
  Eigen::Quaternion<double> integrateAngularVelocity(const Eigen::Quaternion<double>& current_orientation,
                                                     const Eigen::Vector3d& angular_velocity,
                                                     const double& time_step, bool in_body_coordinates)
  {
    Eigen::Quaternion<double> q_current = current_orientation;    q_current.normalize();
    if (q_current.w() < 0.0) { q_current.w() *= -1.0; q_current.vec() *= -1.0; }

    Eigen::Vector3d current_rotation = logarithmicMap(q_current);
    Eigen::Vector3d current_rotation_rate = fromAngularVelocity(current_rotation, angular_velocity, in_body_coordinates);

    // return exponentialMap(current_rotation + current_rotation_rate*time_step);
    return exponentialMap(current_rotation_rate*time_step) * current_orientation;
  }

  // error in the orientation between current and desired orientation
  Eigen::Vector3d orientationError(const Eigen::Quaternion<double>& desired_orientation,
                                   const Eigen::Quaternion<double>& current_orientation)
  {
    Eigen::Quaternion<double> q_desired = desired_orientation;    q_desired.normalize();
    Eigen::Quaternion<double> q_current = current_orientation;    q_current.normalize();
    if (q_current.dot(q_desired) < 0.0) { q_current.w() *= -1.0; q_current.vec() *= -1.0; }

    Eigen::Quaternion<double> delta_quaternion = q_desired*q_current.inverse();
    Eigen::Vector3d delta_rotation = logarithmicMap(delta_quaternion);
    return delta_rotation;
  }

}
