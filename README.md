# On Time Optimization of Centroidal Momentum Dynamics

This software provides methods for optimization of time and momentum of centroidal momentum dynamics for humanoid robots. The repository is self-contained, it provides an implementation of an interior point solver for second-order cone programs based on Embedded Conic Solver (https://github.com/embotech/ecos) using sparse matrix routines (http://www.suitesparse.com).

## Dependencies

For usage on ubuntu 16.04 / 14.04, you are invited to check the Dockerfile in the docker folder, to either check the required dependencies or to generate a docker image in which instructions give on this page work out of the box.

## Getting started
Clone the repository in the desired work folder <work_folder>
```
git clone …
```

Compile the code, by running the following commands in the <work_folder>
```
cd timeoptimization
./src/catkin/third_party/catkin/bin/catkin_make
```

Once the code has been compiled, you can source the setup.bash file
```
source ./devel/setup.bash
```

## Running tests
By running the tests, you can make sure, the software is properly running on your computer. It has been tested on Ubuntu 14.04 and macOS Sierra 10.12.6.
```
catkin_make run_tests_solver
catkin_make run_tests_momentumopt
```

## Running a demo and visualising results

The package “momentumopt” contains within its config folder, configuration files for all demos. Each configuration file contains all information required for an optimization, namely: robot initial state, a predefined contact sequence, robot parameters (e.g. foot support area, robot mass, length of endeffectors), weights for the dynamics optimization, and a set of default parameters for the solver. In order to run a demo, you need to go to the binaries folder for the momentumopt package, and run the demo passing as an argument a name of a config_file. For example:

```
cd ./devel/lib/momentumopt
./demo_momentumopt -i <name_of_cfg_file_within_config_folder>

```

Finally, to visualize the results a script has been provided. The script can be found in the momentumopt package, within the scripts folder. To use it, you can run the following command to open python (Anaconda 2.7) and display optimization results there.

```
python <path_to_momentumopt>/scripts/display.py -i <path_to_momentumopt>/config/<name_of_cfg_file_within_config_folder>_results.yaml
```

Results are stored in a file of the same name as the config file, but appended the word “_results”

License
----

The software is distributed under the [GNU General Public License v3.0](http://www.gnu.org/copyleft/gpl.html).

Credits
----

+ Max Planck Society (developer of time and momentum optimization)
+ Alexander Domahidi (developer of interior point solver)
+ Timothy A. Davis (developer of sparse matrix routines)
