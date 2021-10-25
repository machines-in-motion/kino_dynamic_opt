/**
 * @file PyParams.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>

#include <momentumopt/setting/PlannerParams.hpp>

namespace py = pybind11;
using namespace momentumopt;

void init_params(py::module &m)
{
  // binding integer parameters
  py::enum_<PlannerIntParam>(m, "PlannerIntParam")
  .value("PlannerIntParam_NumDofs", PlannerIntParam_NumDofs)
  .value("PlannerIntParam_NumTimesteps", PlannerIntParam_NumTimesteps)
  .value("PlannerIntParam_NumViapoints", PlannerIntParam_NumViapoints)
  .value("PlannerIntParam_NumJointViapoints", PlannerIntParam_NumJointViapoints)
  .value("PlannerIntParam_NumBaseViapoints", PlannerIntParam_NumBaseViapoints)
  .value("PlannerIntParam_NumJointViapoints_Second", PlannerIntParam_NumJointViapoints_Second)
  .value("PlannerIntParam_NumBaseViapoints_Second", PlannerIntParam_NumBaseViapoints_Second)
  .value("PlannerIntParam_NumJointViapoints_Nonlinear", PlannerIntParam_NumJointViapoints_Nonlinear)
  .value("PlannerIntParam_NumBaseViapoints_Nonlinear", PlannerIntParam_NumBaseViapoints_Nonlinear)
  .value("PlannerIntParam_InverseKinematicsSolver", PlannerIntParam_InverseKinematicsSolver)
	.value("PlannerIntParam_NumActiveDofs", PlannerIntParam_NumActiveDofs)
	.value("PlannerIntParam_NumSubsamples", PlannerIntParam_NumSubsamples)
	.value("PlannerIntParam_KinDynIterations", PlannerIntParam_KinDynIterations)
	.value("PlannerIntParam_MaxNumTimeIterations", PlannerIntParam_MaxNumTimeIterations)
	.value("PlannerIntParam_NumExtendedActiveDofs", PlannerIntParam_NumExtendedActiveDofs)
	.value("PlannerIntParam_NumActiveEndeffectors", PlannerIntParam_NumActiveEndeffectors)
	.value("PlannerIntParam_MaxKinConvergenceIterations", PlannerIntParam_MaxKinConvergenceIterations)
  .export_values();

  // binding boolean parameters
  py::enum_<PlannerBoolParam>(m, "PlannerBoolParam")
  .value("PlannerBoolParam_StoreData", PlannerBoolParam_StoreData)
	.value("PlannerBoolParam_DisplayMotion", PlannerBoolParam_DisplayMotion)
	.value("PlannerBoolParam_LoadKinematics", PlannerBoolParam_LoadKinematics)
  .value("PlannerBoolParam_IsTimeHorizonFixed", PlannerBoolParam_IsTimeHorizonFixed)
  .value("PlannerBoolParam_IsFrictionConeLinear", PlannerBoolParam_IsFrictionConeLinear)
	.value("PlannerBoolParam_UseDefaultSolverSetting", PlannerBoolParam_UseDefaultSolverSetting)
  .export_values();

  // binding double parameters
  py::enum_<PlannerDoubleParam>(m, "PlannerDoubleParam")
  .value("PlannerDoubleParam_Gravity", PlannerDoubleParam_Gravity)
  .value("PlannerDoubleParam_TimeStep", PlannerDoubleParam_TimeStep)
	.value("PlannerDoubleParam_RobotMass", PlannerDoubleParam_RobotMass)
	.value("PlannerDoubleParam_FloorHeight", PlannerDoubleParam_FloorHeight)
	.value("PlannerDoubleParam_RobotWeight", PlannerDoubleParam_RobotWeight)
  .value("PlannerDoubleParam_TimeHorizon", PlannerDoubleParam_TimeHorizon)
	.value("PlannerDoubleParam_MinRelHeight", PlannerDoubleParam_MinRelHeight)
	.value("PlannerDoubleParam_WeightArmTorque", PlannerDoubleParam_WeightArmTorque)
	.value("PlannerDoubleParam_WeightLegTorque", PlannerDoubleParam_WeightLegTorque)
	.value("PlannerDoubleParam_MassTimesGravity", PlannerDoubleParam_MassTimesGravity)
	.value("PlannerDoubleParam_WeightTimePenalty", PlannerDoubleParam_WeightTimePenalty)
	.value("PlannerDoubleParam_FrictionCoefficient", PlannerDoubleParam_FrictionCoefficient)
	.value("PlannerDoubleParam_KinConvergenceTolerance", PlannerDoubleParam_KinConvergenceTolerance)
	.value("PlannerDoubleParam_MaxTimeResidualTolerance", PlannerDoubleParam_MaxTimeResidualTolerance)
	.value("PlannerDoubleParam_WeightTimeRegularization", PlannerDoubleParam_WeightTimeRegularization)
  .value("PlannerDoubleParam_MinTimeResidualImprovement", PlannerDoubleParam_MinTimeResidualImprovement)
  .value("PlannerDoubleParam_SwingTrajViaZ", PlannerDoubleParam_SwingTrajViaZ)
  .value("PlannerDoubleParam_WeightLinMomentumTracking", PlannerDoubleParam_WeightLinMomentumTracking)
  .value("PlannerDoubleParam_WeightAngMomentumTracking", PlannerDoubleParam_WeightAngMomentumTracking)
  .value("PlannerDoubleParam_WeightEndEffContact", PlannerDoubleParam_WeightEndEffContact)
  .value("PlannerDoubleParam_WeightEndEffTracking", PlannerDoubleParam_WeightEndEffTracking)
  .value("PlannerDoubleParam_PGainEndEffTracking", PlannerDoubleParam_PGainEndEffTracking)
  .value("PlannerDoubleParam_PGainComTracking", PlannerDoubleParam_PGainComTracking)
  .value("PlannerDoubleParam_WeightJointReg", PlannerDoubleParam_WeightJointReg)
  .value("PlannerDoubleParam_PGainOrientationTracking", PlannerDoubleParam_PGainOrientationTracking)
  .value("PlannerDoubleParam_PGainPositionTracking", PlannerDoubleParam_PGainPositionTracking)
  .value("PlannerDoubleParam_SwingTrajViaZ_Second", PlannerDoubleParam_SwingTrajViaZ_Second)
  .value("PlannerDoubleParam_WeightLinMomentumTracking_Second", PlannerDoubleParam_WeightLinMomentumTracking_Second)
  .value("PlannerDoubleParam_WeightAngMomentumTracking_Second", PlannerDoubleParam_WeightAngMomentumTracking_Second)
  .value("PlannerDoubleParam_WeightEndEffContact_Second", PlannerDoubleParam_WeightEndEffContact_Second)
  .value("PlannerDoubleParam_WeightEndEffTracking_Second", PlannerDoubleParam_WeightEndEffTracking_Second)
  .value("PlannerDoubleParam_PGainEndEffTracking_Second", PlannerDoubleParam_PGainEndEffTracking_Second)
  .value("PlannerDoubleParam_PGainComTracking_Second", PlannerDoubleParam_PGainComTracking_Second)
  .value("PlannerDoubleParam_WeightJointReg_Second", PlannerDoubleParam_WeightJointReg_Second)
  .value("PlannerDoubleParam_DGainEndEffTracking_Second", PlannerDoubleParam_DGainEndEffTracking_Second)
  .value("PlannerDoubleParam_PGainBaseOrientationTracking_Second", PlannerDoubleParam_PGainBaseOrientationTracking_Second)
  .value("PlannerDoubleParam_DGainBaseOrientationTracking_Second", PlannerDoubleParam_DGainBaseOrientationTracking_Second)
  .value("PlannerDoubleParam_PGainJointRegularization_Second", PlannerDoubleParam_PGainJointRegularization_Second)
  .value("PlannerDoubleParam_DGainJointRegularization_Second", PlannerDoubleParam_DGainJointRegularization_Second)
  .value("PlannerDoubleParam_SwingTrajViaZ_Nonlinear", PlannerDoubleParam_SwingTrajViaZ_Nonlinear)
  .value("PlannerDoubleParam_WeightStance_Nonlinear", PlannerDoubleParam_WeightStance_Nonlinear)
  .value("PlannerDoubleParam_WeightSwingTracking_Nonlinear", PlannerDoubleParam_WeightSwingTracking_Nonlinear)
  .value("PlannerDoubleParam_WeightComTracking_Nonlinear", PlannerDoubleParam_WeightComTracking_Nonlinear)
  .value("PlannerDoubleParam_WeightMomentumTracking_Nonlinear", PlannerDoubleParam_WeightMomentumTracking_Nonlinear)
  .value("PlannerDoubleParam_RatioBasePosReg_Nonlinear", PlannerDoubleParam_RatioBasePosReg_Nonlinear)
  .value("PlannerDoubleParam_RatioBaseOriReg_Nonlinear", PlannerDoubleParam_RatioBaseOriReg_Nonlinear)
  .value("PlannerDoubleParam_RatioJointPosReg_Nonlinear", PlannerDoubleParam_RatioJointPosReg_Nonlinear)
  .value("PlannerDoubleParam_RatioBaseVelReg_Nonlinear", PlannerDoubleParam_RatioBaseVelReg_Nonlinear)
  .value("PlannerDoubleParam_RatioBaseAngVelReg_Nonlinear", PlannerDoubleParam_RatioBaseAngVelReg_Nonlinear)
  .value("PlannerDoubleParam_RatioJointVelReg_Nonlinear", PlannerDoubleParam_RatioJointVelReg_Nonlinear)
  .value("PlannerDoubleParam_WeightStateReg_Nonlinear", PlannerDoubleParam_WeightStateReg_Nonlinear)
  .value("PlannerDoubleParam_WeightControlReg_Nonlinear", PlannerDoubleParam_WeightControlReg_Nonlinear)
  .value("PlannerDoubleParam_WeightTerminalStateReg_Nonlinear", PlannerDoubleParam_WeightTerminalStateReg_Nonlinear)
  .value("PlannerDoubleParam_WeightTerminalControlReg_Nonlinear", PlannerDoubleParam_WeightTerminalControlReg_Nonlinear)
  .export_values();


  // binding string parameters
  py::enum_<PlannerStringParam>(m, "PlannerStringParam")
  .value("PlannerStringParam_ConfigFile", PlannerStringParam_ConfigFile)
  .value("PlannerStringParam_SaveDynamicsFile", PlannerStringParam_SaveDynamicsFile)
	.value("PlannerStringParam_SaveKinematicsFile", PlannerStringParam_SaveKinematicsFile)
	.value("PlannerStringParam_DefaultSolverSettingFile", PlannerStringParam_DefaultSolverSettingFile)
  .export_values();

  // binding Eigen::VectorXi vector parameters
  py::enum_<PlannerIntVectorParam>(m, "PlannerIntVectorParam")
  .value("PlannerIntVectorParam_ActiveDofs", PlannerIntVectorParam_ActiveDofs)
	.value("PlannerIntVectorParam_ExtendedActiveDofs", PlannerIntVectorParam_ExtendedActiveDofs)
	.export_values();

  // binding Eigen::VectorXd vector parameters
  py::enum_<PlannerVectorParam>(m, "PlannerVectorParam")
  .value("PlannerVectorParam_TimeRange", PlannerVectorParam_TimeRange)
  .value("PlannerVectorParam_TorqueRange", PlannerVectorParam_TorqueRange)
	.value("PlannerVectorParam_ExternalForce", PlannerVectorParam_ExternalForce)
	.value("PlannerVectorParam_GravityVector", PlannerVectorParam_GravityVector)
	.value("PlannerVectorParam_CenterOfMassMotion", PlannerVectorParam_CenterOfMassMotion)
	.value("PlannerVectorParam_MaxEndeffectorLengths", PlannerVectorParam_MaxEndeffectorLengths)

  .value("PlannerVectorParam_WeightArmForce", PlannerVectorParam_WeightArmForce)
  .value("PlannerVectorParam_WeightLegForce", PlannerVectorParam_WeightLegForce)
	.value("PlannerVectorParam_WeightArmForceRate", PlannerVectorParam_WeightArmForceRate)
	.value("PlannerVectorParam_WeightLegForceRate", PlannerVectorParam_WeightLegForceRate)
	.value("PlannerVectorParam_WeightCenterOfMass", PlannerVectorParam_WeightCenterOfMass)
	.value("PlannerVectorParam_WeightLinearMomentum", PlannerVectorParam_WeightLinearMomentum)
  .value("PlannerVectorParam_WeightAngularMomentum", PlannerVectorParam_WeightAngularMomentum)
  .value("PlannerVectorParam_WeightLinearMomentumRate", PlannerVectorParam_WeightLinearMomentumRate)
	.value("PlannerVectorParam_WeightAngularMomentumRate", PlannerVectorParam_WeightAngularMomentumRate)
	.value("PlannerVectorParam_WeightFinalLinearMomentum", PlannerVectorParam_WeightFinalLinearMomentum)
	.value("PlannerVectorParam_WeightCenterOfMassViapoint", PlannerVectorParam_WeightCenterOfMassViapoint)
	.value("PlannerVectorParam_WeightFinalAngularMomentum", PlannerVectorParam_WeightFinalAngularMomentum)
	.value("PlannerVectorParam_WeightDynamicTrackingLinearMomentum", PlannerVectorParam_WeightDynamicTrackingLinearMomentum)
	.value("PlannerVectorParam_WeightDynamicTrackingAngularMomentum", PlannerVectorParam_WeightDynamicTrackingAngularMomentum)

  .value("PlannerVectorParam_MinJointAngles", PlannerVectorParam_MinJointAngles)
	.value("PlannerVectorParam_MaxJointAngles", PlannerVectorParam_MaxJointAngles)
	.value("PlannerVectorParam_KinematicDefaultJointPositions", PlannerVectorParam_KinematicDefaultJointPositions)

	.value("PlannerVectorParam_WeightJointVelocity", PlannerVectorParam_WeightJointVelocity)
	.value("PlannerVectorParam_WeightJointAcceleration", PlannerVectorParam_WeightJointAcceleration)
	.value("PlannerVectorParam_WeightKinematicTrackingCenterOfMass", PlannerVectorParam_WeightKinematicTrackingCenterOfMass)
	.value("PlannerVectorParam_WeightKinematicDefaultJointPositions", PlannerVectorParam_WeightKinematicDefaultJointPositions)
	.value("PlannerVectorParam_WeightKinematicTrackingLinearMomentum", PlannerVectorParam_WeightKinematicTrackingLinearMomentum)
	.value("PlannerVectorParam_WeightKinematicTrackingAngularMomentum", PlannerVectorParam_WeightKinematicTrackingAngularMomentum)
	.value("PlannerVectorParam_WeightKinematicTrackingBaseOrientation", PlannerVectorParam_WeightKinematicTrackingBaseOrientation)
	.value("PlannerVectorParam_WeightKinematicTrackingLinearMomentumRate", PlannerVectorParam_WeightKinematicTrackingLinearMomentumRate)
	.value("PlannerVectorParam_WeightKinematicTrackingAngularMomentumRate", PlannerVectorParam_WeightKinematicTrackingAngularMomentumRate)
	.value("PlannerVectorParam_WeightKinematicTrackingEndeffectorPosition", PlannerVectorParam_WeightKinematicTrackingEndeffectorPosition)

  .value("PlannerVectorParam_PGainMomentumTracking_Second", PlannerVectorParam_PGainMomentumTracking_Second)
  .export_values();

  // binding c-vector variables used by the planner
  py::enum_<PlannerCVectorParam>(m, "PlannerCVectorParam")
    .value("PlannerCVectorParam_JointViapoints", PlannerCVectorParam_JointViapoints)
    .value("PlannerCVectorParam_BaseViapoints", PlannerCVectorParam_BaseViapoints)
    .value("PlannerCVectorParam_JointViapoints_Second", PlannerCVectorParam_JointViapoints_Second)
    .value("PlannerCVectorParam_BaseViapoints_Second", PlannerCVectorParam_BaseViapoints_Second)
    .value("PlannerCVectorParam_JointViapoints_Nonlinear", PlannerCVectorParam_JointViapoints_Nonlinear)
    .value("PlannerCVectorParam_BaseViapoints_Nonlinear", PlannerCVectorParam_BaseViapoints_Nonlinear)
    .value("PlannerCVectorParam_Viapoints", PlannerCVectorParam_Viapoints)
    .export_values();
  }
