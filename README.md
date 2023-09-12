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

______
## Our projects

### Active SLAM, indoor scenes data collection, and dynamic SLAM

With this framework in conjuction with our [people generator](https://github.com/eliabntt/animated_human_SMPL_to_USD), [environment exporter](https://github.com/eliabntt/Front3D_to_USD) and [control framework](https://github.com/eliabntt/ros_isaac_drone) (which can control any thanks to our [custom 6DOF joint controller](https://github.com/eliabntt/custom_6dof_joint_controller)), we generated an extensive dataset of indoor animated scenes.

The data generated has been then post-processed and evaluated with our set of [tools](https://github.com/robot-perception-group/GRADE_tools) against popular SLAM libraries, and used to test the realism your synthetic data.

With those tests we showed how many of these methods cannot recover from failures, and have highly degraded performance in dynamic environments even during very short sequences(60 seconds).

### In the wild Zebras observed by drones

We used the teleport capabilities of the system to generate both an **outdoor synthetic Zebra** datasets. The details are in the corresponding [Zebra](https://arxiv.org/abs/2305.00432) paper. The goal was to try to bridge the gap between simulation and reality and demonstrate that we can avoid tedious tasks such as precise data annotation.

Using a variety of environments from Unreal Engine and a freely available zebra model we were able to generate data realistic enough to obtain models trained from *scratch* that reached >90% accuracy on real world data.

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
## HowToS, Requirements, Installation and Known issues

Please follow [this](https://github.com/eliabntt/GRADE-RR/blob/main/HOWTO.md) link.

______
## Download data

The data will be available in our [data repository](https://github.com/eliabntt/GRADE_data/).

__________
## Citations

If you find this work useful please cite our work

1. GRADE: currently under revision. Please cite this if you use this code, ideas from this paper, etc.
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

2. Synthetic Zebras: Published at ECMR2023. Please cite this if you use the data or the models published related to this publication, or if you find this work inspiring.
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

3. Dyanmic SLAM evaluations: published at the Active Vision for Robotics Workshop at ICRA 2023. Please cite this with GRADE main work if you want to reference the evaluations we did on Dynamic SLAM.

```
@inproceedings{ bonetto2023dynamicSLAM, 
            title={{S}imulation of {D}ynamic {E}nvironments for {SLAM}}, 
            author={Elia Bonetto and Chenghao Xu and Aamir Ahmad}, 
            booktitle={ICRA2023 Workshop on Active Methods in Autonomous Navigation}, 
            year={2023}, 
            url={https://arxiv.org/abs/2305.04286}}
```

4. Detection and segmentation of humans in indoor scenes using synthetic data: published at the Pretraining for Robotics workshop at ICRA 2023. Please cite this with GRADE main work if you find our work on humans detection inspiring. 

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

