# Kino-dynamic Trajectory Optimization for Quadruped Robot

This software provides methods for kino-dynamic optimization for multiped robots.

## Getting started
Clone the repository in the desired work folder <work_folder>
```
git clone …
```

Compile the code, by running the following commands in the <work_folder>
```
cd kino-dynamic-opt
./src/catkin/third_party/catkin/bin/catkin_make
```

Once the code has been compiled, you can source the setup.bash file
```
source ./devel/setup.bash
```

## Running a demo 
```
cd <work_folder>/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos
python3 ./PyDemoMomentumopt.py -i <path_to_config_file>
```
For example plan and execute a jumping motion with
```
python3 ./PyDemoMomentumopt.py -i ../config/cfg_quadruped_jump.yaml
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

For an explanation of the different settings in the configuration files, refer to cfg_quadruped_jump.yaml.

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
python3 ./PyDemoMomentumopt.py -i <path_to_config_file>
```
automatically saves the quadruped_positions.dat and the quadruped_velocities.dat files in the <work_folder>/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos directory. These files can then be supplied to dynamic graph and for example executed on the real robot.  