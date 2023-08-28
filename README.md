# GRADE-RR or how to Generate Realistic Animated Dynamic Environments for Robotics Research

## Isaac 2022.2.1 branch is partially updated --- The repeating experiments, the utils and the FUEL_simulation scripts should work as intended

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

If you want more information check out my [GTC talk](https://www.nvidia.com/gtc/session-catalog/?search=bonetto&tab.catalogallsessionstab=16566177511100015Kus&search=bonetto#/session/1666623015127001DShI), the [paper](https://arxiv.org/abs/2303.04466) or our [website](https://eliabntt.github.io/grade-rr).

GRADE can work in conjuction with our [people generator](https://github.com/eliabntt/animated_human_SMPL_to_USD), [environment exporter](https://github.com/eliabntt/Front3D_to_USD) and [control framework](https://github.com/eliabntt/ros_isaac_drone) (which can control any robot thanks to our [custom 6DOF joint controller](https://github.com/eliabntt/custom_6dof_joint_controller)).

The data generated can be post-processed with our set of [tools](https://github.com/robot-perception-group/GRADE-eval), [evaluated](https://github.com/robot-perception-group/GRADE-eval) against popular SLAM libraries, and used to test the realism your synthetic data by training for example [Yolo and MaskRCNN](https://github.com/eliabntt/GRADE-train).

We used this project to generate both an **indoor dynamic environment** and a **outdoor synthetic Zebra** datasets. The details for those are in the corresponding [GRADE](https://arxiv.org/abs/2303.04466) and [Zebra](https://arxiv.org/abs/2305.00432) papers. To do so, we used [this](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/zebra_datagen.py) main file.

In this readme I'll give the main installation instructions, and links to other resources that you should read to work with this software.

The main branch (as of now) contains code compatible with the 2021.2.1 version of the code.
For the v2022.2.1 please checkout the corresponding branch

_______
## List of project-related repositories

1. All the data we generated (indoor, zebras, additional scenarios), the TUM-RGBD fr3/walking sequence labelled, the networks checkpoints trained as described in the paper, the rosbags used to evaluate the SLAM methods and the results will be available [here](https://github.com/eliabntt/GRADE_data/)
2. The tools to process the data, add noise to the rosbags or during the simulation, to evaluate the SLAM methods, generate training data can be found [here](https://github.com/robot-perception-group/GRADE_tools)
3. The code to convert SMPL-based animations to USD files is [here](https://github.com/eliabntt/animated_human_SMPL_to_USD)
4. To convert any environment from Blender to USD and generate some accompanying data use [this](https://github.com/eliabntt/Front3D_to_USD). This has a special focus in indoor environmets and Front3D. Based on BlenderProc.
5. The parent repository which we used to autonomously explore the environments during the data generation is [here](https://github.com/eliabntt/ros_isaac_drone)
6. The modified version of DynaSLAM working with Python3 and using `detectron2` is [here](https://github.com/eliabntt/DynaSLAM)
___________________

### [Install](https://github.com/eliabntt/GRADE-RR/blob/main/Install.md)

______________
### Misc

Every script has been commented and almost each piece of code should be self-explanatory. If you don't find it like that please open an issue.

Also, we welcome any contribution that you might have. Include coding style (I'm no SE), comments, additions, or better strategies that you want to propose (of course after you have published your paper).

Finally, in these readmes we will try to stay general. Please check out each code piece for parameters and understand how it works. 

__________

### How to run the simulation

[Here](https://github.com/eliabntt/GRADE-RR/blob/main/SAMPLES.md) you will learn what we already tried out, what we tested, what we used to run the simulations. In there, there is also a quick plug-and-play example. [Here](https://github.com/eliabntt/GRADE-RR/blob/main/OUR_CODE.md) you can learn about our developed codebase, where you can find useful resources, and how you can edit them, file by file. Also [here](https://github.com/eliabntt/GRADE-RR/blob/main/OUR_CODE.md) you can find instructions on how to easily setup your first simulation by following `simulator/simulator_ros.py` code. More informations are also located in each sub-repository of this project.

### How to postprocess the data

Please check our dedicated repository [here](https://github.com/robot-perception-group/GRADE_tools).

### How to colorize the saved data

Simply run `python scripts/colorize.py --viewport_folder main_folder_with_npy_files`.
Check our code [here](https://github.com/eliabntt/GRADE-RR/blob/main/scripts/colorize.py), you can save images, images and videos, and decide which kind of data you want.

### How to get skeletal, vertices, and SMPL information while correcting bounding boxes

Look [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/smpl_and_bbox.py). This is mainly tuned for our data. However, it can be easily expanded to your own dataset. A new script dedicated with more information will be updated soon.

### How to edit directly USD files

Check the tutorial [here](https://github.com/eliabntt/GRADE-RR/blob/main/EDIT_USDS.md). This will help you convert USD to txt files for easy file processing.

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

### Segmentation <-> instance

Instance segmentation files will save also the mappings between classes. An example on how to do the mapping and process those file is [here](https://github.com/robot-perception-group/GRADE-eval/blob/main/mapping_and_visualization/convert_classes.py).

_____
## Known issues
1. ros clock might have some delay in publishing. This implies that you need to sleep the simulation every time that component gets triggered. Other component behave consistently based on our tests. Alternatively, you can post-process the data as shown in [here](https://github.com/robot-perception-group/GRADE-eval)
2. BBOX3D are wrong for moving objects. The script in `scripts/smpl_and_bbox.py` will solve this.
3. Collisions for dynamic objects are wrong most of the times. This implies that LiDAR (and LRF) will get wrong data when running. This should be addressed by the new LiDAR-RTX of the new Isaac Sim version.
4. The rendering is not blocking. Multiple calls (especially for path tracing) are necessary. However, this usually disrupt the motion-vector data and motion-blur. A possible workaround is [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/replay_experiment.py) on our replay-experiment script.

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
3. Dyanmic SLAM evaluations: published at the Active Methods in Autonomous Navigation Workshop at ICRA 2023
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

