# GRADE-RR or how to Generate Realistic Animated Dynamic Environments for Robotics Research
## Project developed with Isaac Sim 2021.2.1
#### Isaac 2022 branch is being updated --- All the commands will work equally between the two versions.

This repository contains the code of GRADE.

GRADE is a system I developed to seamlessly manage the Isaac Sim simulation software to Generate Realistic Animated Dynamic Environments for Robotics Research


![prom](https://user-images.githubusercontent.com/19806758/215789830-c6dfd612-0b35-4640-b3fb-b5abbe877dee.png)


This will help you in:
1. managing the simulation
2. load, placem, animate assets
3. load and control any robot
4. get sensor readings from such robots, saving ground truth data
5. customize your workflow
6. postprocess the data
7. repeat any experiment --- *this includes recording new sensor, getting new data, changing the conditions and repair the data while working in realistically looking environments and in a physics enabled simulator.*

Each step of the pipeline can be easily customized, expanded or removed from your workflow.

If you want more information check out my [GTC talk](), the [paper]() or our [website](https://eliabntt.github.io/grade-rr).

With this framework in conjuction with our [people generator](https://github.com/eliabntt/animated_human_SMPL_to_USD), [environment exporter](https://github.com/eliabntt/Front3D_to_USD) and [control framework](https://github.com/eliabntt/ros_isaac_drone) (which can control any thanks to our [custom 6DOF joint controller](https://github.com/eliabntt/custom_6dof_joint_controller)), we generated a dataset.

The dataset has been then postprocessed with our set of [tools](https://github.com/robot-perception-group/GRADE-eval), [evaluated](https://github.com/robot-perception-group/GRADE-eval) against popular SLAM libraries and used to test the realism of our synthetic data by [training Yolo and MaskRCNN](https://github.com/eliabntt/GRADE-train).

In this readme I'll give the main installation instructions, and links to other resources that you should read to work with this software.

The main branch (for now) contains code compatible with the 2021.2.1 version of the code. Some modifications are necessary for upgrading to the 2022 version and we are working on that.
_______
## List of project-related repositories

1. All the data we generated, the TUM labelled data, the networks checkpoints trained as described in the paper, the rosbags used to evaluate the SLAM methods and the results will be available [here](https://github.com/eliabntt/GRADE_data/)
2. The tools to process the data, add noise to the rosbags or during the simulation, to evaluate the SLAM methods, generate training data can be found [here](https://github.com/robot-perception-group/GRADE_tools)
3. [here](https://github.com/eliabntt/GRADE-nets) you can find the Mask RCNN and YOLOv5 networks we used to train our data, plus some additional helping script.
4. The code to convert SMPL-based animations to USD files is [here](https://github.com/eliabntt/animated_human_SMPL_to_USD)
5. To convert any environment from Blender to USD and generate some accompanying data use [this](https://github.com/eliabntt/Front3D_to_USD). This has a special focus in indoor environmets and Front3D. Based on BlenderProc.
6. The parent repository which we used to autonomously explore the environments during the data generation is [here](https://github.com/eliabntt/ros_isaac_drone)
7. The modified version of DynaSLAM working with Python3 and using `detectron2` is [here](https://github.com/eliabntt/DynaSLAM)
___________________

## Requirements and basic software installation

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
├── SAMPLES.md # how to run the samples
├── MOVEMENT.md # how can you control the camera/robot?
├── PARAMS.md # available (and expandable) parameters description
├── README.md # main readme
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

______________
## Misc

A general note: every script has been commented and almost each piece of code should be self-explanatory. If you don't find it like that please open an issue.

Also, we welcome any contribution that you might have. Include coding style (I'm no SE), comments, additions, or better strategies that you want to propose (of course after you have published your paper).

Finally, in these readmes we will try to stay general. Please check out each code piece for parameters and understand how it works. 

### Ways to run the simulation

[Here](https://github.com/eliabntt/GRADE-RR/blob/main/SAMPLES.md) you will learn what we already tried out, what we tested, what we used to run the simulations. In there, there is also a quick plug-and-play example. [Here](https://github.com/eliabntt/GRADE-RR/blob/main/OUR_CODE.md) you can learn about our developed codebase, where you can find useful resources, and how you can edit them, file by file. Also [here](https://github.com/eliabntt/GRADE-RR/blob/main/OUR_CODE.md) you can find instructions on how to easily setup your first simulation by following `simulator/simulator_ros.py` code. More informations are also located in each sub-repository of this project.

### How to postprocess the data

Please check our dedicated repository [here](https://github.com/robot-perception-group/GRADE_tools).

### How to colorize the saved data

Simply run `python scripts/colorize.py --viewport_folder main_folder_with_npy_files`.
Check our code [here](https://github.com/eliabntt/GRADE-RR/blob/main/scripts/colorize.py), you can save images, images and videos, and decide which kind of data you want.

### How to get skeletal, vertices, and SMPL information while correcting bounding boxes

Look [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/smpl_and_bbox.py). This is mainly tuned for our data. However, it can be easily expanded to your own dataset.

### How to edit directly USD files

Check the tutorial [here](https://github.com/eliabntt/GRADE-RR/blob/main/EDIT_USDS.md). This will help you convert USD to txt files for easy file processing.

### Isaac's edited files

Edited files are inside `isaac_internals`. The edited ones are the one that are copied by the `cp_local..` script. However, as per Isaac requirements, we had to include all the licenses and other files.

- _Shapenet_ minor edits regarding the main script since the dowload website seem down. We suggest to pre-download the dataset, unpack it, and set-up the environment folders as we show [here](https://github.com/eliabntt/GRADE-RR/blob/main/OUR_CODE.md) to directly use the pre-downloaded data.
- _synthetic\_recorder_ created a custom extension to save our data, and offset the number of cameras. In that way we can save high-resolution images to the disk, while providing ROS smaller images. We found this faster than resizing images afterwards and caused less "issues".
- _synthetic\_utils_ we edited the `numpy.py` and the `syntheticdata.py` to save more data and have more flexibility. What is still missing (our bad) is the vertical fov of the camera, which is not directly exposed by Isaac Sim.
- In `setup_python_env.sh` we had to prevent the loading of `$SCRIPT_DIR/exts/omni.isaac.motion_planning/bin` (you can find it commented at the very end of line 8), to be able to run the system version of `move_base`. That module could be necessary for some of the Isaac extensions or configurations. Please be aware of this.

### How to move/control the camera/robot

You have several possibilities with and without ROS, with and without physics. Check them out [here](https://github.com/eliabntt/GRADE-RR/blob/main/MOVEMENT.md)

### Additionally core developed/edited packages related to the project

1. `FUEL`, our chosen autonomous exploration manager to control the drone within the environment. [Link here](https://github.com/eliabntt/FUEL/tree/main)
2. `custom_6dof_joint_controller` which is the bridge between the position/velocity commands and the joint velocities expected by IsaacSim. This will allow you to control any robot within the simulation environment. [Link here](https://github.com/eliabntt/custom_6dof_joint_controller/tree/main)
3. `moveit_based_collision_checker_and_placement` our Move-it based placement strategy. [Link here](https://github.com/eliabntt/moveit_based_collision_checker_and_placement/tree/main)

### Possible missing textures/wrong paths

When loading humans or environments (or anything else) it will be necessar for you to edit the paths of the shaders, especially when moving between Windows and Linux.
To do that you can use the `change_shader_path` (line: 59, `misc_utils.py`) or the `correct_paths` (line: 123, `misc_utils.py`).
Please note that you need to tailor this to your own use case.
The `correct_paths` function is already commented in `human_utils` when loading the human and in `environment_utils` when loading the env. 

Otherwise, you can simply process the text files as explained [here](https://github.com/eliabntt/GRADE-RR/blob/main/EDIT_USDS.md).

### Code explanation
Most of the functions in the utils libraries are commented and explained. If something is unclear feel free to open an issue.

### Segmentation <-> instance

Instance segmentation files will save also the mappings between classes. An example on how to do the mapping and process those file is [here](https://github.com/robot-perception-group/GRADE-eval/blob/main/mapping_and_visualization/convert_classes.py).

_____
## Known issues
1. ros clock might have some delay in publishing. This implies that you need to sleep the simulation every time that component gets triggered. Other component behave consistently based on our tests. Alternatively, you can post-process the data as shown in [here](https://github.com/robot-perception-group/GRADE-eval)
2. BBOX3D are wrong for moving objects. The script in `scripts/smpl_and_bbox.py` will solve this.
3. Collisions for dynamic objects are wrong most of the times. This implies that LiDAR (and LRF) will get wrong data when running. This should be addressed by the new LiDAR-RTX of the new Isaac Sim version.
4. The rendering is not blocking. Multiple calls (especially for path tracing) are necessary. However, this usually disrupt the motion-vector data and motion-blur. A possible workaround is [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/replay_experiment.py) on our replay-experiment script.

______
## Download data

The data will be available in our [data repository](https://github.com/eliabntt/GRADE_data/).

__________
### CITATION
If you find this work useful please cite our work as

```

```

______
## Thanks

I would like to thank the amazing [NVIDIA support](http://forums.developer.nvidia.com) for their quick response times and precise answers.
[Chenghao Xu](http://kyle-xu-001.github.io/) for helping in testing and refining the evaluation scripts. [Aamir Ahmad](aamirahmad.de) for his supervision.
