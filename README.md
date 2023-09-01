# GRADE-RR or how to Generate Realistic Animated Dynamic Environments for Robotics Research

### Note that while we used the v2021 for the paper, that version is now deprecated. I will work only on v2022+

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

If you want more information check out the [paper](https://arxiv.org/abs/2303.04466) or our [website](https://eliabntt.github.io/grade-rr).

With this framework in conjuction with our [people generator](https://github.com/eliabntt/animated_human_SMPL_to_USD), [environment exporter](https://github.com/eliabntt/Front3D_to_USD) and [control framework](https://github.com/eliabntt/ros_isaac_drone) (which can control any thanks to our [custom 6DOF joint controller](https://github.com/eliabntt/custom_6dof_joint_controller)), we generated a dataset of indoor animated scenes.

The data generated can be post-processed with our set of [tools](https://github.com/robot-perception-group/GRADE-eval), [evaluated](https://github.com/robot-perception-group/GRADE-eval) against popular SLAM libraries, and used to test the realism your synthetic data.

We used this project to generate both an **indoor dynamic environment** and a **outdoor synthetic Zebra** datasets. The details for those are in the corresponding [GRADE](https://arxiv.org/abs/2303.04466) and [Zebra](https://arxiv.org/abs/2305.00432) papers. 

_______
## List of project-related repositories

1. All the data we generated is or will be available [here](https://github.com/eliabntt/GRADE_data/)
2. The tools to process the data, add noise to the rosbags or during the simulation, to evaluate the SLAM methods, generate training data can be found [here](https://github.com/robot-perception-group/GRADE_tools)
3. The code to convert SMPL-based animations to USD files is [here](https://github.com/eliabntt/animated_human_SMPL_to_USD)
4. To convert any environment from Blender to USD and generate some accompanying data use [this](https://github.com/eliabntt/Front3D_to_USD). This has a special focus in indoor environmets and Front3D. Based on BlenderProc.
5. The parent repository which we used to autonomously explore the environments during the data generation is [here](https://github.com/eliabntt/ros_isaac_drone)
6. The modified version of DynaSLAM working with Python3 and using `detectron2` is [here](https://github.com/eliabntt/DynaSLAM)
7. `FUEL`, our chosen autonomous exploration manager to control the drone within the environment. [Link here](https://github.com/eliabntt/FUEL/tree/main)
8. `custom_6dof_joint_controller` which is the bridge between the position/velocity commands and the joint velocities expected by IsaacSim. This will allow you to control any robot within the simulation environment. [Link here](https://github.com/eliabntt/custom_6dof_joint_controller/tree/main)
9. `moveit_based_collision_checker_and_placement` our Move-it based placement strategy. [Link here](https://github.com/eliabntt/moveit_based_collision_checker_and_placement/tree/main)
___________________

## Requirements and basic software installation

Please check the [requirements](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/requirements.html) on the official page.

Install Nucleus, Cache, and Isaac Sim.

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

A general note: every script has been more or less commented and almost each piece of code should be self-explanatory. If you don't find it like that please **open an issue**.

I worked on this mainly alone so the code is far from perfect, super-modular, or anything like that. But together we can make it better. 

Thus, we welcome any contribution that you might have. Include coding style, comments, additions, or better strategies that you want to propose (of course after you have published your paper).

### Ways to run the simulation

We're working on this piece of the documentation. Please bear with us while we upgrade the documents with better instructions.

For now, [here](https://github.com/eliabntt/GRADE-RR/blob/v2022/SAMPLES.md) you will learn what we already tried out, what we tested, what we used to run the simulations. In there, there is also a quick plug-and-play example. [Here](https://github.com/eliabntt/GRADE-RR/blob/v2022/OUR_CODE.md) you can learn about our developed codebase, where you can find useful resources, and how you can edit them, file by file. Also [here](https://github.com/eliabntt/GRADE-RR/blob/v2022/OUR_CODE.md) you can find instructions on how to easily setup your first simulation by following `simulator/simulator_ros.py` code. More informations are also located in each sub-repository of this project.

### How to postprocess the data

Please check our dedicated repository [here](https://github.com/robot-perception-group/GRADE_tools).

### How to colorize the saved data

Simply run `python scripts/colorize.py --viewport_folder main_folder_with_npy_files`.
Check our code [here](https://github.com/eliabntt/GRADE-RR/blob/v2022/scripts/colorize.py), you can save images, images and videos, and decide which kind of data you want.

### How to get skeletal, vertices, and SMPL information while correcting bounding boxes

Look [here](https://github.com/eliabntt/GRADE-RR/blob/v2022/simulator/smpl_and_bbox.py). This is mainly tuned for our data. However, it can be easily expanded to your own dataset.

### How to edit directly USD files

Check the tutorial [here](https://github.com/eliabntt/GRADE-RR/blob/v2022/EDIT_USDS.md). This will help you convert USD to txt files for easy file processing.

### Isaac's edited files

Edited files are inside `isaac_internals`. The edited ones are the one that are copied by the `cp_local..` script. However, as per Isaac requirements, we had to include all the licenses and other files.

- _Shapenet_ minor edits regarding the main script since the dowload website seem down. We suggest to pre-download the dataset, unpack it, and set-up the environment folders as we show [here](https://github.com/eliabntt/GRADE-RR/blob/v2022/OUR_CODE.md) to directly use the pre-downloaded data.
- _synthetic\_recorder_ created a custom extension to save our data, and offset the number of cameras. In that way we can save high-resolution images to the disk, while providing ROS smaller images. We found this faster than resizing images afterwards and caused less "issues".
- _synthetic\_utils_ we edited the `numpy.py` and the `syntheticdata.py` to save more data and have more flexibility. What is still missing (our bad) is the vertical fov of the camera, which is not directly exposed by Isaac Sim.
- In `setup_python_env.sh` we had to prevent the loading of `$SCRIPT_DIR/exts/omni.isaac.motion_planning/bin` (you can find it commented at the very end of line 8), to be able to run the system version of `move_base`. That module could be necessary for some of the Isaac extensions or configurations. Please be aware of this.

### How to move/control the camera/robot

You have several possibilities with and without ROS, with and without physics. Check them out [here](https://github.com/eliabntt/GRADE-RR/blob/v2022/MOVEMENT.md)

### Possible missing textures/wrong paths

When loading humans or environments (or anything else) it may be necessar for you to edit the paths of the shaders, especially when moving between Windows and Linux.
To do that you can use the [`change_shader_path`](https://github.com/eliabntt/GRADE-RR/blob/v2022/simulator/utils/misc_utils.py#L62) or the [correct paths](https://github.com/eliabntt/GRADE-RR/tree/v2022/scripts/process_paths) scripts.

Otherwise, you can simply process the text files as explained [here](https://github.com/eliabntt/GRADE-RR/blob/v2022/EDIT_USDS.md).

### Segmentation <-> instance

Instance segmentation files will save also the mappings between classes. An example on how to do the mapping and process those file is [here](https://github.com/robot-perception-group/GRADE-eval/blob/main/mapping_and_visualization/convert_classes.py).

_____
## Known issues
1. ros clock might have some delay in publishing. This implies that you need to sleep the simulation every time that component gets triggered. Other component behave consistently based on our tests. Alternatively, you can post-process the data as shown in [here](https://github.com/robot-perception-group/GRADE-eval)
2. BBOX3D are wrong for moving objects. The script [here](https://github.com/eliabntt/GRADE-RR/blob/v2022/simulator/correct_data.py#L267) show a way to solve this.
3. Pose information is wrong for some moving objects. The code [here](https://github.com/eliabntt/GRADE-RR/blob/v2022/simulator/correct_data.py#L224) will solve this.
4. Collisions for dynamic objects are not computed most of the times due to PhysX limitations. This is addressed by the new LiDAR-RTX of the new Isaac Sim version.
5. The rendering is not blocking. Multiple calls (especially for path tracing) are necessary. Thus, this usually disrupt the motion-vector data. A possible workaround is to do two rendering steps and save the motion-vector data, and then finish rendering to save the rgb information. See [here](https://github.com/eliabntt/GRADE-RR/blob/v2022/simulator/replay_experiment.py#L390) an example on how to do that.
6. In the v2022 it is not possible to set indipendent vfov of the cameras
7. In the v2022 the internal PD control for the joints will NOT work using position setpoints. Also, the maximum velocity set is not considered.
8. In the v2022 the timeline gets updated automatically even if you do not want it. You need to keep track of the ctime and constantly re-update it to correctly generate the data you want.

______
## Download data

The data will be available in our [data repository](https://github.com/eliabntt/GRADE_data/).

__________
### CITATION
If you find this work useful please cite our work

1. GRADE: currently under revision
```
@misc{bonetto2023grade,
  doi = {10.48550/ARXIV.2303.04466},
  url = {https://arxiv.org/abs/2303.04466},
  author = {Bonetto, Elia and Xu, Chenghao and Ahmad, Aamir},
  keywords = {Robotics (cs.RO), FOS: Computer and information sciences, FOS: Computer and information sciences},
  title = {GRADE: Generating Realistic Animated Dynamic Environments for Robotics Research},
  publisher = {arXiv},
  year = {2023},
  copyright = {arXiv.org perpetual, non-exclusive license}
}
```
2. Synthetic zebras: currently under revision
```
@misc{bonetto2023synthetic,
      title={Synthetic Data-based Detection of Zebras in Drone Imagery}, 
      author={Elia Bonetto and Aamir Ahmad},
      year={2023},
      publisher = {arXiv},
      url = {https://arxiv.org/abs/2305.00432},
      doi = {10.48550/arXiv.2305.00432},
      primaryClass={cs.CV}
}
```
3. Dyanmic SLAM evaluations: published at the Active Vision for Robotics Workshop at ICRA 2023
```
@inproceedings{ bonetto2023dynamicSLAM, 
            title={{S}imulation of {D}ynamic {E}nvironments for {SLAM}}, 
            author={Elia Bonetto and Chenghao Xu and Aamir Ahmad}, 
            booktitle={ICRA2023 Workshop on Active Methods in Autonomous Navigation}, 
            year={2023}, 
            url={https://arxiv.org/abs/2305.04286}}
```
4. Detection and segmentation of humans in indoor scenes using synthetic data: published at the Pretraining for Robotics workshop at ICRA 2023
```
@inproceedings{
bonetto2023learning,
title={Learning from synthetic data generated with {GRADE}},
author={Elia Bonetto and Chenghao Xu and Aamir Ahmad},
booktitle={ICRA2023 Workshop on Pretraining for Robotics (PT4R)},
year={2023},
url={https://openreview.net/forum?id=SUIOuV2y-Ce}
}
```
____________

## LICENSE
By downloading and/or using the Data & Software (including downloading, cloning, installing, and any other use of the corresponding github repository), you acknowledge that you have read these terms and conditions, understand them, and agree to be bound by them. If you do not agree with these terms and conditions, you must not download and/or use the Data & Software. Any infringement of the terms of this agreement will automatically terminate your rights under this License

Accompanying software, such as, but not limited to, the one from Isaac Sim, is licensed according to their specific term of use.

If you use data/software from other projects such as, but not limited to, TUM RGB-D, 3D-Front, 3D-Future, ... it is your responsibility to follow their licensing terms.

If you have questions regarding the license, please contact the [ps-licensing@tue.mpg.de](mailto:ps-licensing@tue.mpg.de).
______
## Thanks

I would like to thank the amazing [NVIDIA support](http://forums.developer.nvidia.com) for their quick response times and precise answers.
[Chenghao Xu](http://kyle-xu-001.github.io/) for helping in testing and refining the evaluation scripts. [Aamir Ahmad](aamirahmad.de) for his supervision.

