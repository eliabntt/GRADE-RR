# GRADE-RR or how to Generate Real Life Animated Dynamic Environments for Robotics Research
## Project developed with Isaac Sim 2021.2.1
#### Isaac 2022 branch is being updated

This repository contains the code of GRADE.

GRADE is a system I developed to seamlessly manage the Isaac Sim simulation software.

This will help you in:
1. managing the simulation
2. load assets
3. place assets
4. animate assets
5. load any robot
6. control any robot/camera however you want
7. get sensor readings from such robots, saving ground truth data
8. customize your workflow
9. postprocess the data
10. repeat any experiment --- # this implies recording new sensor, getting new data, changing the conditions and repair the data
while working in realistically looking environments and in a physics enabled simulator.

Each step of the pipeline can be easily customized, expanded or removed from your workflow.

If you want more information check out my [GTC talk](), the [paper]() or our [website](https://eliabntt.github.io/grade).

With this framework in conjuction with our [people generator](https://github.com/eliabntt/generate_people), [environment exporter](https://github.com/eliabntt/BlenderProc/tree/working_branch) and [control framework](https://github.com/eliabntt/ros_isaac_drone) (which can control any thanks to our [custom 6DOF joint controller](https://github.com/eliabntt/custom_6dof_joint_controller)), we generated a dataset.

The dataset has been then postprocessed with our set of [tools](https://github.com/robot-perception-group/GRADE-eval) and evaluated against popular SLAM libraries and used to test the realism of our synthetic data.

In this readme I'll give the main installation instructions, and links to other resources that you should read to work with this software.

The main branch (for now) contains code compatible with the 2021.2.1 version of the code. Some modifications are necessary for upgrading to the 2022 version and we are working on that.

### Requirements and basic software installation

Please check the [requirements](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/requirements.html) on the official page.

Install Nucleus, Cache, and Isaac Sim.

For 2021 version use the following as Nucleus server in the omniverse launcher:

```
Name: Isaac
Type: Amazon S3
Host: d28dzv1nop4bat.cloudfront.net
Service: s3
Redirection: https://d28dzv1nop4bat.cloudfront.net
```

For 2022 this should not be necessary.

From now on, we will assume that you installed Isaac Sim within a `ISAAC_FOLDER`. Default location is `~/.local/share/ov/pkg/isaac-version/`.

Clone this repository. You can clone this wherever you prefer. For simplicity, we usually download it within the `isaac` folder.
However, by using global paths you should be able to run this code anywhere in your PC.

_Note_ Isaac will have its own python installation, if you need packages and you run software within the Isaac python executable remember that. To do so, you usually do something like
```
cd $ISAAC_FOLDER
./python.sh -m pip install ...
# or
./python.sh python_file.py
```

We have some dependencies which are not installed by default. To install them run `sh req.sh $ISAAC_FOLDER`. (This will simply use the main Isaac `python.sh` to install everything via `pip`).

### Folder structure

```bash
├── EDIT_USDS.md # how to convert USDs to text files and edit them
├── HOW_IT_WORKS.md # main "HOWTO" file
├── MOVEMENT.md # how can you control the camera/robot?
├── PARAMS.md # available (and expandable) parameters description
├── README.md # main readme
├── STRIP_ROS.md # how can you run this without ros
├── cp_local_to_diff_folder.sh # update code from/to isaac folder
├── irotate_specific # specific files used for simulate irotate in isaac sim and instructions
│   └── ...
├── isaac_internals # edited isaac files
│   ├── apps
│   │   └── omni.isaac.sim.python.kit # pre-load some additional extensions and disable a moveit (so that we can load the one from the system)
│   ├── exts 
│   │   ├── omni.isaac.shapenet # slightly modified loader
│   │   ├── omni.isaac.synthetic_recorder # custom recorder extension that allows more control
│   │   └── omni.isaac.synthetic_utils # minor edits
│   └── setup_python_env.sh # source the ros environment and show how to source multiple ones
├── kill.sh # script to kill the whole simulation
├── meshes # folder containing meshes
├── req.sh # requirements file
├── scripts # useful scripts
│   ├── bash_process.zsh # multiprocessing procedure (zsh shell)
│   ├── colorize.py # colorize your data
│   ├── get_benchbot.sh # get benchbot environments
│   └── process_paths # folder containing script to automatically process USD files (see EDIT_USD.md file)
├── simulator # main simulator folder, each main file will have it's own description
│   ├── configs # yaml configuration files
│   ├── utils # utils loaded and used by the main files
│   └── ... 
└── usds # usds files
```

### Finishing setting up

Independently on where you cloned the repository you need to run
`sh cp_local_to_different_folder.sh $CLONE_FOLDER $ISAAC_FOLDER`

This will copy the edited files from $1 (source) to the $2 (destination). You can use it in reverse (from Isaac to repo), or with any couple of folders.

### Ways to run the simulation

We have several showcase examples (all located in the simulator folder).
To run the code the general process is `./python.sh python_script args`. Each one of the python files has its own configuration yaml file. We will give example launch commands for each one of these.

0. `simple` this is a simple piece of code with which you can load the environment, the robot and start simulating. Find more info [here]().
1. `paper_simulation` this is the code that we used to generate the dataset. You can find more informations [here](). This requires ROS, some other package to manage the autonomous exploration. This will show you how to control your ROS modules from Isaac, how you can tick components, how you can trigger other modules, how you can save data, how you can load objects and humans. Note that this is a quite complex file (which should be probably re-written).
2. `simulator_ros` a simpler version of 1. Check it [here](). This is practically the same thing, without the tie to FUEL and other software. Some other option is included.  
3. `irotate_simulation` this is the code that we used to simulate [iRotate](https://github.com/eliabntt/irotate_active_slam), our active SLAM method, with Isaac Sim. You can find more informations [here](). This is very similar to 1 and 2, despite using an initial location but shwos how you can manage different robot with practically the same code. 
3. `multi_robot_sim` simulate multi robots, a bit hardcoded but generalizable. You can find more info [here](). This simulate two drones and a ground robot. The two drones will be controlled independently with two FUEL sessions, while the ground robot is controlled with iRotate.
4. `savana_simulation` to show how we created the Savana with the Zebras. You can find more info [here](). Animated animals are pre-positioned within the environment. The robot is controlled through joint waypoints.
5. `replay_experiment` how one can exactly replay the experiment to expand it. Get more info [here](). You can see how teleport works, how internal joint commands can work and how you can reload a USD file of an experiment, its configuration, and while modifying the robot or the environment, replay it.

Each simulation file will power up the environment, load the assets and manage the saving based on the loaded configuration file.

### How to postprocess the data

Please check our dedicated repository [here](https://github.com/robot-perception-group/GRADE-eval).

### How to colorize the saved data

Simply run `python scripts/colorize.py --viewport_folder main_folder_with_npy_files`.
Check our code [here](https://github.com/eliabntt/GRADE-RR/blob/main/scripts/colorize.py), you can save images, images and videos, and decide which kind of data you want.

### How to get skeletal, vertices, and SMPL information while correcting bounding boxes

Look [here](https://github.com/eliabntt/GRADE-RR/blob/main/scripts/smpl_and_bbox.py). This is mainly tuned for our data. However, it can be easily expanded to your own dataset.

### What did we edit

Edited files are inside `isaac_internals`. The edited ones are the one that are copied by the `cp_local..` script. However, as per Isaac requirements, we had to include all the licenses and other files.

- _Shapenet_ minor edits regarding the main script since the dowload website seem down. We suggest to pre-download the dataset, unpack it, and set-up the environment folders as we show [here]() to directly use the pre-downloaded data.
- _synthetic\_recorder_ created a custom extension to save our data, and offset the number of cameras. In that way we can save high-resolution images to the disk, while providing ROS smaller images. We found this faster than resizing images afterwards and caused less "issues".
- _synthetic\_utils_ we edited the `numpy.py` and the `syntheticdata.py` to save more data and have more flexibility. What is still missing (our bad) is the vertical fov of the camera, which is not directly exposed by Isaac Sim.
- In `setup_python_env.sh` we had to prevent the loading of `$SCRIPT_DIR/exts/omni.isaac.motion_planning/bin` (you can find it commented at the very end of line 8), to be able to run the system version of `move_base`. That module could be necessary for some of the Isaac extensions or configurations. Please be aware of this.

#### ROS-packages \[if desired\]

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

### ROS-independence

The main things dependent on ROS are:
1. The placement procedure (look for `position_object`, you can override/edit it how you like)
2. The ROS components attached to the robot/camera. Depending on which level of independence you want you might disable everything or keep the joint and tf publishers. Note that every viewport is a burden on the system. Also, every time you publish camera data through ROS there is some overhead.

### Movement strategies

You have several possibilities. 

With ROS:
1. Use a 6 joints mechanism as we do. Implement your own and loop it back through ROS as we have done with `FUEL`. You can edit the `custom_joint_6dof_controller` or directly publish `joint_commands`. This is doable even directly within the main simulation loop as it is setted up to already be a node. You can manage this from the main simulation loop (as in `paper_simulation`) or from another program as we do with `iRotate`
2. Use an embedded controller provided by IssacSim and publish `cmd_vel` or `moveit` commands dependending on your use case.
3. Launch a python script through the Isaac `python.sh` and use ROS and `joint_commands` to read the robot location (listen to the `joint_states` or odom topics) and follow your path.

Without ROS:
1. Move the robot by sending joint position/velocity commands directly from IsaacSim. An example of this has been implemented on the `Savana` branch. This will output physics and will abide the settings that you use for your joints (mass, force...). 
2. Use a strategy like the one we use for the [flying objects](https://github.com/eliabntt/isaac_sim_manager/blob/main/simulator/objects_utils.py#L166). However, this does NOT include physics, collision or anything similar whatsoever. In this case the trajectory is followed blindly and interpolated based on your settings.
3. Use the IsaacSim KeyFrame extension visually to manually set up 2. beforehand.

## How to run

At this point, from the ISAAC folder you can run 
```
./python.sh simulator/simulator_ros.py [--/renderer/enabled='rtx,iray'] --neverending=True --rtx_mode=True --record=False  --config="/GLOBAL/simulator/config.yaml" 
```

`python.sh` will load all the necessary packages
`simulator/simulator_ros.py` is the path of the main python.
`--/renderer/enabled='rtx,iray'` makes sure that `iray` is enabled. You can safely remove this.

All the parameters are optional with the exception of the config file.
If you remove something from the config file please be sure that is not used in the code. Otherwise, it will crash. 

You can see a detailed explanation of the parameters and of the config file [here](https://github.com/eliabntt/isaac_sim_manager/blob/main/PARAMS.md).

*BASH PROCESSING*
If you want to run everything (including the exploration visualization and the rosbag recorder) the `bash_process.zsh` file is what you are looking for.
That file is what we used to streamline the generation and process in batches. In the config file you can easily chose which sensor to use.
Be aware that `motion-vector` is NOT working (IsaacSim bug) and that enabling everything will save a _lot_ of data.


## How to anonymize? How to convert USD binary to TEXT? How to change paths _before_ loading the `usd` file?

Install `USD` package from [here](https://github.com/PixarAnimationStudios/USD/).
Then take the saved usd and do:
`mv original.usd original.usdc`
`usdcat -o original_text.usda original.usdc`
Then you can process `original_text.usda` with a text editor/python script/whatever you like.

## Possible missing textures/wrong paths

When loading humans or environments (or anything else) it might happen that you need to edit the paths of the shaders, especially when moving between Windows and Linux.
To do that you can use the `change_shader_path` (line: 56, `utils.py`) or the `correct_paths` (line: 111, `utils.py`). <!---### TODO put links--->
Please note that you need to tailor this to your own use case.
The `correct_paths` function is already commented in `human_utils` when loading the human and in `environment_utils` when loading the env. 

Otherwise, you can simply process the text files as explained above and load the new ones.

### Code explanation
Most of the functions in the utils libraries are commented and explained. If something is unclear feel free to open an issue.

### Segmentation <-> instance

Instance segmentation files will save also the mappings between classes. An example on how to do the mapping and process those file is [here](https://github.com/robot-perception-group/GRADE-eval/blob/main/mapping_and_visualization/convert_classes.py).