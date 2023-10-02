# GRADE-RR or how to Generate Realistic Animated Dynamic Environments for Robotics Research

### Note that while we used the v2021 for the paper, that version is now deprecated. I will work only on v2022+

GRADE is a system I developed to seamlessly manage the Isaac Sim simulation software to Generate Realistic Animated Dynamic Environments for Robotics Research

![prom](NewCover.png)

This will help you in:
1. managing the simulation
2. load, place, animate assets
3. load and control any robot --- with or without ROS, with or without physics
4. get sensor readings from such robots, saving *ground truth* or *noisy* data
5. customize your workflow
6. postprocess the data --- add noise, reorganize the bags, prep the data for DL models...
7. repeat any experiment --- *this includes recording new sensor, getting new data, changing the conditions and repair the data while working in realistically looking environments and in a physics enabled simulator.*


Each step of the pipeline can be easily customized, expanded or removed from your workflow.


If you want more information check out the [paper](https://arxiv.org/abs/2303.04466) or our [website](https://eliabntt.github.io/grade-rr).

_______
## Useful related repositories (that couldn't fit this page)

1. The tools to process the data, add noise to the rosbags or during the simulation, to evaluate the SLAM methods, generate training data can be found [here](https://github.com/robot-perception-group/GRADE_tools)
2. The code to convert SMPL-based animations to USD files is [here](https://github.com/eliabntt/animated_human_SMPL_to_USD). Use this if you want to convert AMASS animated SMPL models, the Cloth3D dataset, or any other dataset that you might have that contains skeletal animations. If you use something different than SMPL (or some of its variations), you will need to extend this code.
3. To convert any environment from Blender to USD and generate some accompanying data use [this](https://github.com/eliabntt/Front3D_to_USD). This has a special focus in indoor environmets and Front3D. Based on BlenderProc. You can use this tool also to convert ANY fbx or other file.
4. The tools we used to autonomously explore the environments during the data generation is [here](https://github.com/eliabntt/ros_isaac_drone), using RotorS, FUEL, our custom 6DOF controller, etc.
5. The modified version of DynaSLAM working with Python3 and using `detectron2` is [here](https://github.com/eliabntt/DynaSLAM)
6. `custom_6dof_joint_controller` is the bridge between the position/velocity commands and the joint velocities expected by IsaacSim. This will allow you to control any robot within the simulation environment. [Link here](https://github.com/eliabntt/custom_6dof_joint_controller/tree/main). 
7. `moveit_based_collision_checker_and_placement` our Move-it based placement strategy. [Link here](https://github.com/eliabntt/moveit_based_collision_checker_and_placement/tree/main)

______
## Our projects

### Active SLAM, indoor scenes data collection, and dynamic SLAM

With this framework in conjuction with our [people generator](https://github.com/eliabntt/animated_human_SMPL_to_USD), [environment exporter](https://github.com/eliabntt/Front3D_to_USD) and [control framework](https://github.com/eliabntt/ros_isaac_drone) (which can control virtually anything thanks to our expandable [custom 6DOF joint controller](https://github.com/eliabntt/custom_6dof_joint_controller)), we generated an extensive dataset of indoor animated scenes.

The data generated has been then post-processed and evaluated with our set of [tools](https://github.com/robot-perception-group/GRADE_tools) against popular SLAM libraries, and used to test the realism your synthetic data.

With those tests we showed how many of these methods cannot recover from failures, and have highly degraded performance in dynamic environments even during very short sequences(60 seconds).

### In the wild Zebras observed by drones

We used the teleport capabilities of the system to generate both an **outdoor synthetic Zebra** datasets. The details are in the corresponding [Zebra](https://arxiv.org/abs/2305.00432) paper. The goal was to try to bridge the gap between simulation and reality and demonstrate that we can avoid tedious tasks such as precise data annotation.

Using a variety of environments from Unreal Engine and a freely available zebra model we were able to generate data realistic enough to obtain models trained from *scratch* that reached >90% accuracy on real world data.

_______

### Folder structure

<details closed>
<summary>A folder structure summary with comments of what is inside each folder</summary>

```bash
├── cp_local_to_diff_folder.sh # update code from/to isaac folder
├── irotate_specific # specific files used for simulate irotate in isaac sim and instructions
│   └── ...
├── isaac_internals # edited isaac files
│   ├── apps
│   │   └── omni.isaac.sim.python.kit # pre-load some additional extensions and disable a moveit (so that we can load the one from the system)
│   ├── kit # solve some bugs in the synthetic data processing
│   ├── exts 
│   │   ├── omni.isaac.shapenet # slightly modified loader
│   │   ├── omni.isaac.synthetic_recorder # custom recorder extension that allows more control
│   │   └── omni.isaac.synthetic_utils # minor edits
│   └── setup_python_env.sh # source the ros environment and show how to source multiple ones
├── kill.sh # script to kill the whole simulation
├── req.sh # requirements file
├── scripts # useful scripts and additional accompanying stuff
│   └── ...
├── simulator # main simulator folder, each main file will have it's own description
│   ├── configs # yaml configuration files
│   ├── utils # utils loaded and used by the main files
│   └── ... 
├── meshes # folder containing meshes
└── usds # usds files
```

</details closed>

___________________

## HowToS, Installation, Tips, and Known issues

The system, contrary to Gazebo, is not straightforward. This is the price you have to pay to be able to access low level APIs and have more control. We highly encourage thorugh readings of the documentation, of the tips section, and for you to get acquainted to the utils that we have organized (perhaps badly, open a pull request please).

[Install, StartUp, Issues](https://github.com/eliabntt/GRADE-RR/blob/main/HOWTO.md)

[Tips](https://github.com/eliabntt/GRADE-RR/blob/main/TipsAndTricks.md) --- highly encouraged reading!

To [generate people based on SMPL](https://github.com/eliabntt/animated_human_SMPL_to_USD), [convert environments/objects from Front3D or other files beforehand](https://github.com/eliabntt/Front3D_to_USD) and see a possible [control framework](https://github.com/eliabntt/ros_isaac_drone) (which can act thanks to our [custom 6DOF joint controller](https://github.com/eliabntt/custom_6dof_joint_controller)), please check our other repositories. 

Additional scripts are provided [here](https://github.com/eliabntt/GRADE-RR/blob/main/scripts). Those can be used to process paths, get statistics of the rosbags, colorize the data filter and compress rosbags, transform the pixels to world coordinates etc.

A brief description of the utils libraries used in our code is [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/utils/UTILS.md).

_____
## Isaac's edited files details

<details closed>
<summary>We had to edit some of the files to have more flexibility and solve some bugs. Here are reported details</summary>

Edited files are inside `isaac_internals`. The edited ones are the one that are copied by the `cp_local..` script. As per Isaac requirements, we had to include all the licenses and other files. Note that these might be outdated w.r.t. your current installation.

- _synthetic\_recorder_ created a custom extension to save our data, and offset the number of cameras. In that way we can save high-resolution images to the disk, while providing ROS smaller images. We found this faster than resizing images afterwards and caused less "issues".
- _synthetic\_utils_ we edited the `numpy.py` and the `syntheticdata.py` to save more data and have more flexibility. What is still missing (our bad) is the vertical fov of the camera, which is not directly exposed by Isaac Sim.
- In `setup_python_env.sh` we had to prevent the loading of `$SCRIPT_DIR/exts/omni.isaac.motion_planning/bin` (you can find it commented at the very end of line 8), to be able to run the system version of `move_base`. That module could be necessary for some of the Isaac extensions or configurations. Please be aware of this.
- `apps/omni.isaac.sim.python.kit` will load a couple of additional necessary extensions
- `isaac_internals/kit/extscore/omni.syntheticdata` will simply solve some bugs related to out of bounds and processing errors

</details closed>

______
## Download data

The data will be available in our [data repository](https://github.com/eliabntt/GRADE_data/).

__________
## Citations

You acknowledge that the Data & Software is a valuable scientific resource and agree to appropriately reference the following paper in any publication making use of the Data & Software.

Citation:

```
@misc{bonetto2023grade,
            doi = {10.48550/ARXIV.2303.04466},
            url = {https://arxiv.org/abs/2303.04466},
            author = {Bonetto, Elia and Xu, Chenghao and Ahmad, Aamir},
            title = {GRADE: Generating Realistic Animated Dynamic Environments for Robotics Research},
            publisher = {arXiv},
            year = {2023},
            copyright = {arXiv.org perpetual, non-exclusive license}
}
```

Additionally:

- If you use any Data and/or Software related to zebras(animal) detection from drone imagery reference the following paper in any publication as well
```
@INPROCEEDINGS{10256293,
  author={Bonetto, Elia and Ahmad, Aamir},
  booktitle={2023 European Conference on Mobile Robots (ECMR)}, 
  title={Synthetic Data-Based Detection of Zebras in Drone Imagery}, 
  year={2023},
  volume={},
  number={},
  pages={1-8},
  doi={10.1109/ECMR59166.2023.10256293}}
}
```

- If you use any Data and/or Software related to our Dyanmic SLAM evaluations
```
@inproceedings{bonetto2023dynamicSLAM, 
            title={{S}imulation of {D}ynamic {E}nvironments for {SLAM}}, 
            author={Elia Bonetto and Chenghao Xu and Aamir Ahmad}, 
            booktitle={ICRA2023 Workshop on Active Methods in Autonomous Navigation}, 
            year={2023}, 
            url={https://arxiv.org/abs/2305.04286},
            month = jun,
            month_numeric = {6}
}
```

- If you use any Data and/or Software related to the tasks of detection/segmentation of humans in dynamic environments.
```
@inproceedings{bonetto2023learning,
            title={Learning from synthetic data generated with {GRADE}},
            author={Elia Bonetto and Chenghao Xu and Aamir Ahmad},
            booktitle={ICRA2023 Workshop on Pretraining for Robotics (PT4R)},
            year={2023},
            url={https://openreview.net/forum?id=SUIOuV2y-Ce},
            month = jun,
            month_numeric = {6}
}
```
____________

## LICENSE
By downloading and/or using the Data & Software (including downloading, cloning, installing, and any other use of the corresponding github repository), you acknowledge that you have read these terms and conditions, understand them, and agree to be bound by them. If you do not agree with these terms and conditions, you must not download and/or use the Data & Software. Any infringement of the terms of this agreement will automatically terminate your rights under this License. Please read the [licensing](https://github.com/eliabntt/GRADE-RR/blob/main/LICENSE.md) agreement prior to any use of our Data or Software.

Accompanying software, such as, but not limited to, the one from Isaac Sim, is licensed according to their specific term of use.

If you use data/software from other projects such as, but not limited to, TUM RGB-D, 3D-Front, 3D-Future, ... it is your responsibility to follow their licensing terms, whose you implicitly agree.

If you have questions regarding the license, please contact the [ps-licensing@tue.mpg.de](mailto:ps-licensing@tue.mpg.de).
______
## Thanks

I would like to thank the amazing [NVIDIA support](http://forums.developer.nvidia.com) for their quick response times and precise answers.
[Chenghao Xu](http://kyle-xu-001.github.io/) for helping in testing and refining the evaluation scripts. [Aamir Ahmad](aamirahmad.de) for his supervision.

