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

/**
 * This demo shows how to use the dynamics optimization routine given
 * an appropriate configuration file. It requires an initial robot state
 * and a reference momentum trajectory. The optimized motion plan is available
 * to the user under a DynamicsSequence object (dyn_optimizer.dynamicsSequence()),
 * which is a collection of dynamic states, each of which contains information
 * about all variables, and all end-effectors of the motion plan for one time step.
 *
 * If the variable store_data within the configuration file has been set
 * to True, then a file of results will also be written. It can be found
 * next to the configuration file with the keyword "_results" appended.
 * It can be visualized with the available python script as indicated in
 * the Readme file.
 */

#include <iomanip>
#include <yaml-cpp/yaml.h>
#include <momentumopt/dynopt/DynamicsOptimizer.hpp>
#include <momentumopt/cntopt/ContactPlanFromFile.hpp>

using namespace momentumopt;

int main( int argc, char *argv[] )
{
  // load configuration file
  std::string cfg_file = CFG_SRC_PATH;
  if (argc==3) {
    if (std::strcmp(argv[1], "i"))
      cfg_file += argv[2];
  } else {
	std::cout << "Usage: ./demo -i <name of config file within config folder>" << std::endl;
    return 1;
  }

  // define problem configuration
  PlannerSetting planner_setting;
  planner_setting.initialize(cfg_file);

  // define robot initial state
  DynamicsState ini_state;
  ini_state.fillInitialRobotState(cfg_file);

  // define reference kinematic sequence
  momentumopt::KinematicsSequence kin_sequence;
  kin_sequence.resize(planner_setting.get(PlannerIntParam_NumTimesteps),
                      planner_setting.get(PlannerIntParam_NumDofs));

  // define terrain description
  momentumopt::TerrainDescription terrain_description;
  terrain_description.loadFromFile(cfg_file);

  // define contact plan
  ContactPlanFromFile contact_plan;
  contact_plan.initialize(planner_setting);
  contact_plan.optimize(ini_state, terrain_description);

  // optimize motion
  DynamicsOptimizer dyn_optimizer;
  dyn_optimizer.initialize(planner_setting);
  dyn_optimizer.optimize(ini_state, &contact_plan, kin_sequence);

  /*  optimized variables can be retrieved as (remember that forces are normalized by mass times gravity)
   *
   *  for (int time_id=0; time_id<dyn_optimizer.dynamicsSequence().size(); time_id++) {
   *    std::cout << dyn_optimizer.dynamicsSequence().dynamicsState(time_id).centerOfMass().transpose() << std::endl;
   *    std::cout << dyn_optimizer.dynamicsSequence().dynamicsState(time_id).linearMomentum().transpose() << std::endl;
   *    std::cout << dyn_optimizer.dynamicsSequence().dynamicsState(time_id).angularMomentum().transpose() << std::endl;
   *    for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
   *      std::cout << dyn_optimizer.dynamicsSequence().dynamicsState(time_id).endeffectorForce(eff_id).transpose() << std::endl;
   *      std::cout << dyn_optimizer.dynamicsSequence().dynamicsState(time_id).endeffectorTorque(eff_id).transpose() << std::endl;
   *      std::cout << dyn_optimizer.dynamicsSequence().dynamicsState(time_id).endeffectorCoP(eff_id).transpose() << std::endl;
   *    }
   *  }
   */

  /*  An iterative optimization between kinematics and dynamics could be done as follows.
   *  Instead of only running once the optimize function from dynamics_optimizer, it is possible to iterate
   *
   *  dyn_optimizer.optimize(kin_optimizer_.kinematicsSequence());
   *  for (int iter_id=0; iter_id<NumKinDynIterations; iter_id++) {
   *    kin_optimizer_.optimize(dyn_optimizer_.dynamicsSequence());
   *    dyn_optimizer_.optimize(kin_optimizer_.kinematicsSequence(), true);
   *  }
   *
   *  where dynamicsSequence() and kinematicsSequence() are instances of the class DynamicsSequence.
   *  The dynamics_optimizer will try to track the momentum values contained within kinematicsSequence(),
   *  and the kinematics_optimizer will try to track the momentum values contained within dynamicsSequence().
   *  The kinematics_optimizer is not provided.
   *
   *  The "true" flag passed to the dynamics_optimizer signals that it should switch from
   *  regularization weights to tracking weights.
   */

}
