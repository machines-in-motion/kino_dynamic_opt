# Kino-dynamic Trajectory Optimization for Quadruped Robot

This software provides methods for kino-dynamic optimization for multiped robots.

## Install required packages

Please make sure to install the following packages:

- the qp solver quadprog and the code manager
```
pip install quadprog treep
```

And catkin: https://docs.ros.org/api/catkin/html/
- Ubuntu
```
sudo apt-get install [ros-distro]-catkin
```
- From source
```
mkdir devel
cd devel
git clone https://github.com/ros/catkin.git
cd catkin
mkdir _build
cd _build
cmake ..
make
sudo make install
```

## Getting started

Clone the repository in the desired work folder <work_folder>
```
mkdir devel
cd devel
git clone git@git-amd.tuebingen.mpg.de:amd-clmc/treep_amd_clmc.git
git clone git@github.com:machines-in-motion/treep_machines_in_motion.git
treep --clone KINO_DYN_PLANNER
```
This operation should have cloned pacakges in `workspace/src/` which will be
refered as the <work_folder>\
Compile the code, by running the following commands in the <work_folder>
```
cd workspace
catkin_make -DCMAKE_BUILD_TYPE=RELEASE
```

Once the code has been compiled, you can source the setup.bash file in
`devel/workspace/devel/setub.bash`
```
source ./devel/setup.bash
```

## Running a demo 
```
cd <work_folder>/catkin/control/momentumopt/demos
python3 ../nodes/kino_dyn_planner_solo -i <path_to_config_file>
```
For example plan and execute a jumping motion with
```
python3 ../nodes/kino_dyn_planner_solo -i ../config/cfg_quadruped_jump.yaml
```
alternatives:

```
python2 ../nodes/kino_dyn_planner_solo -i ../config/cfg_quadruped_jump.yaml
```
```
rosrun momentumopt kino_dyn_planner_solo -i ../config/cfg_quadruped_jump.yaml
```

#### Configuration overview
The different configuration files are available in 
```
<work_folder>/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/config
```
The available motions are:
* Squatting: cfg_quadruped_squatting.yaml
* Lifting one leg: cfg_quadruped_lift_leg.yaml
* Lifting rear legs: cfg_quadruped_lift_rear.yaml
* Jump: cfg_quadruped_jump.yaml

For an explanation of the different settings in the configuration files, refer to [cfg_quadruped_jump.yaml](/src/catkin/motion_planning/momentumopt/config/cfg_quadruped_jump.yaml).

## Experimental Section
Furthermore, there is a kinematic optimization over a horizon available, which allows for smoother trajectories, by running
```
cd <work_folder>/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos
python3 ./PyDemoMomentumoptNew.py -i <path_to_config_file>
```
For example plan and execute lifting the rear leg motion with
```
python3 ./PyDemoMomentumoptNew.py -i ../config/cfg_demo01_twofeet.yaml
```
#### Configuration overview
The available motions are:
* Lifting rear legs: cfg_demo01_twofeet.yaml

## Saving .dat file for dynamic graph

Executing
```
python3 ../nodes/kino_dyn_planner_solo -i <path_to_config_file>
```
automatically saves the `quadruped_positions.dat` and the `quadruped_velocities.dat` files in the `<work_folder>/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos` directory. These files can then be supplied to dynamic graph and for example executed on the real robot.

## License

Copyright (c) 2019, New York University and Max Planck Gesellschaft.

## Authors

- Brahayam Ponton (<bponton@tue.mpg.de>) (1)
- Majid Khadiv (<mkhadiv@tuebingen.mpg.de>) (1)
- Julian Viereck (<jviereck@tuebingen.mpg.de>) (1)
- Avadesh Meduri (<am9789@nyu.edu>) (1-2)

(1): Max Planck Institute for Intelligent System, Tubingen, Germany \
(2): New York University, New York, USA
