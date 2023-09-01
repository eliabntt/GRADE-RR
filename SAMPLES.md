# HOW TO RUN THE SIMULATION - SAMPLES AND TUTORIAL

We have several showcase examples (all located in the simulator folder).
To run the code the general process is `./python.sh python_script args`. Each one of the python files has its own configuration yaml file. More details will be given below for each file

To follow these tutorials, we suggest that either you download one of our example environments [here]() and human animations [here]() or you use our code [SMPL to USD](https://github.com/eliabntt/animated_human_SMPL_to_USD) and [Blender to USD](https://github.com/eliabntt/Front3D_to_USD) to create your own assets.

We also suggest that you pre-install the [drone](https://github.com/eliabntt/ros_isaac_drone) control and placement repository. 
This is necessary to be able to use our placement strategy, control the drone with our custom 6DOF controller, or use FUEL with IsaacSim.

Each simulation file will power up the environment, load the assets and manage the saving based on the loaded configuration file.

The scripts are the following:

0. `startup_your_own` starting with GRADE. Load a basic scene, a robot, add the sensors, publish the data.
1. `insert_people` add people into the simulation, including placement.
2. `w_objects` add flying objects into the simulation, including their animation.
3. `simulator_ros`. Combine 0,1,2 into a single loop to generate data.
4. `irotate_simulation` this is the code that we used to simulate [iRotate](https://github.com/eliabntt/irotate_active_slam), our active SLAM method, with Isaac Sim. This is very similar to 1 and 2, despite using an initial location but shwos how you can manage different robot with practically the same code.
5. `multi_robot_sim` simulate multi robots, a bit hardcoded but generalizable. This simulate two drones and a ground robot. The two drones will be controlled independently with two FUEL sessions, while the ground robot is controlled with iRotate.
6. `savana_simulation` to show how we created the Savana with the Zebras. Animated animals are pre-positioned within the environment. The robot is controlled through joint waypoints. **THIS DOES NOT WORK in v2022.2.1 DUE TO ISAACSIM BUGS**
7. `replay_experiment` how one can exactly replay the experiment to expand it. You can see how teleport works, how internal joint commands can work and how you can reload a USD file of an experiment, its configuration, and while modifying the robot or the environment, replay it.
8. `correct_data` and `smpl_and_bbox` show how to access low-level information of the meshes and how it is possible to correct the 3DBbox incorrect information.
9. `FUEL_indoor_simulation` this is the code that we used to generate the dataset.
10. `zebra_datagen` this is the code that we used to generate the data for the Zebra paper.


**Each config needs to be updated with your own paths**

### ROS-independence

The main things dependent on ROS in our work are:
1. The placement procedure (look for `position_object`, you can override/edit it how you like). We use MoveIt FCL library to check for collision. A similar procedure can be used with Trimesh package. Or you can implement your own.
2. The ROS components attached to the robot/camera. Depending on which level of independence you want you might disable everything or keep the joint and tf publishers. Note that every viewport is a burden on the system. Also, every time you publish camera data through ROS there is some overhead since GPU data need to be first transferred to CPU and then published. Writing data is asynchronous so no slowdown there.

### Main concepts

The programs all follow the same structure.
- load the basic kit and start the initial simulation
- load a basic environment with some settings pre-applied (some config changes cannot be made with the code itself)
- load libraries and settings
- load your main environment
- edit the environment
- load the robots
- attach sensors to the robot
- correct the camera fov (bug in Isaac that changes it)
- [optional] load and place humans
- [optional] load and place and animate objects
- setup information recorder
- loop the simulation and publish/write the information when necessary

Every aspect can be personalized or adapted. The basic environment could be your final one, the humans/animations can be present or placed in a different way, robot can have your set of sensors or your own publishing rate.

All this can be specified in the code or in the yaml configuration file.

A detailed analysis of the code can be found [here](https://github.com/eliabntt/GRADE-RR/blob/main/OUR_CODE.md) while the parameters are explained [here](https://github.com/eliabntt/GRADE-RR/blob/main/PARAMS.md)

**Beore launching any simulation you need to start `roscore` preferably with sim time set to true (`rosparam set use_sim_time true`)**

In general, each robot is loaded pre-fixed with the `my_robot_` name, and this applies to each topic that is published from that robot. The exception lies in the `tf` topic, for which we will have a publisher for each robot. Data will be published in ROS and saved as npy files. The npy files, although of a different image ratio, have the same vertical and horizontal aperture. You can disable this behaviour by removing the npy cameras commenting `add_npy_cameras` and lowering the number of camera offset of `_my_recorder`.

___

## Paper(ros) simulation

Install ROS and create a `catkin_ws` and install [this](https://github.com/eliabntt/ros_isaac_drone).

The default location for this installation is `$HOME` (`/home/user/catkin_ws`).

The repo above will install 
1. `FUEL`, our chosen exploration manager
2. `mav_comm` and `mav_control_rw` which are used to control the robot and get velocity commands to follow the path generated by `FUEL`
3. `custom_6dof_joint_controller` which is the bridge between the position/velocity commands and the joint velocities expected by IsaacSim
4. `moveit_based_collision_checker_and_placement` which is needed to do the placement of the "objects"

The [README](https://github.com/eliabntt/ros_isaac_drone/blob/main/README.md) already explicate the dependencies.

If you install it in a different location _update `setup_python_env.sh:2`_ with your new location.

Remember that you can also `source ... --extend` to source different environments in cascade.

At this point, assuming you are locate in the ISAAC folder you can run 
```
./python.sh GRADE-RR/simulator/paper_simulation.py --config="/GLOBAL/GRADE-RR/simulator/configs/config_paper.yaml" 
```
*BASH PROCESSING*
If you want to run everything (including the exploration visualization and the rosbag recorder) the `bash_process.zsh` file is what you are looking for.
That file is what we used to streamline the generation and process in batches. In the config file you can easily chose which sensor to use.

Similarly
```
./python.sh GRADE-RR/simulator/simulator_ros.py --config="/GLOBAL/simulator/configs/config.yaml" 
```
would work. Note that in this case you need to edit both the configs and the code otherwise the robot will not move.

_______
## iRotate simulation

Download and install the iRotate package [here](https://github.com/eliabntt/irotate_active_slam/tree/isaac) from the Isaac branch.

This simulation by default does NOT use animated objects. You can see how one can have a blueprint and quickly edit it based on its own convenience.

_update `setup_python_env.sh:2`_ with your catkin workspace location.

Before launching the simulation you need to open a terminal and run `python[3] irotate_specific/republish_tf.py`

Also, run `irotate` as explained in the repo. A set of commands could be:
```
roslaunch robotino_simulations world.launch
roslaunch robotino_simulations rtabmap.launch delete:=-d
roslaunch active_slam active_node.launch
roslaunch robotino_mpc robotino_mpc.launch
```
Note that we launch the FSM later on.

With iRotate we usually let the robot start from `0,0,0` and `yaw=0`. If you change this, like with the previous work, you need to change the ekfs accordingly.

The transform `world->map` is constant. `map->odom` is done by `rtabmap`. `odom->base_link` is done from the ekfs. 

Isaac is setted up to publish the tfs to the `/tf2` topic. Step 4 is necessary to publish everything back to the `/tf` cleaned up of the ground truth estimation.

The custom joint controller has been updated. You need to be sure you are running the one from irotate repository.
Thus, we need either to build everything in the same workspace or use `source ... --extend` if you are using two workspaces. 
You can eventually change the scripts to have it working how you want.

You can launch an rviz visualization with `rviz -d irotate_specific irotate.rviz`

```
./python.sh GRADE-RR/simulator/irotate_simulation.py --config="/GLOBAL/GRADE-RR/simulator/configs/config_irotate.yaml" 
```

Once the simulation is running, you can launch `roslaunch robotino_fsm robotino_fsm.launch kind:=2 only_last_set:=false pre_fix:=true mid_optimizer:=true weighted_avg:=true robot_odom:=/odometry/filtered cam_odom:=/camera/odometry/filtered`

Note how the topics are stilll without `/my_robot_x`. This should be changed in the EKF formulation.
_______
## Multi robot

For this you need both the irotate repository and the original paper repository. The code will launch first the irotate robot and then two drones. You need to include both workspaces in `setup_python_env.sh:2` using first the _ros_isaac_drone_ and then the _irotate_ workspace (use the `--extend` keyword).

You can follow a similar procedure like the one above to launch `iRotate`.

To run the main simulation 
```
./python.sh GRADE-RR/simulator/multi_robot_sim.py --config="/GLOBAL/GRADE-RR/simulator/configs/config_multi_robot.yaml" 
```

This piece of code show you how multiple robots can be loaded and controlled, how the configuration file can be expanded (e.g. only iRotate's robot has an initial location) and how everything can be customized.

_______
## Savana

Is another simple scenario since everything is managed internally. The animations are already placed within the environment and the robot has pre-defined waypoints. The FSM is internal to the main source code which can be launched with
```
./python.sh GRADE-RR/simulator/savana_simulation.py --config="/GLOBAL/GRADE-RR/simulator/configs/config_savana.yaml" 
```
_______
## Replay experiment

This is a very useful piece of code. You can use this to replay any previously recorded experiment, modify the robot (or the scene conditions) and record new data.

Please run

```
./python.sh GRADE-RR/simulator/replay_experiment.py --experiment_folder FOLDER
```
to do so.

In our code we show how to create a new stereo camera, save previously unsaved data, save motion-vector, and create a LiDAR sensor.
