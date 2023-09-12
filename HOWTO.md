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

### Finishing setting up

Independently on where you cloned the repository you need to run
`sh cp_local_to_different_folder.sh $CLONE_FOLDER $ISAAC_FOLDER`

This will copy the edited files from $1 (source) to the $2 (destination). You can use it in reverse (from Isaac to repo), or with any couple of folders.

______________
## Misc

A general note: every script has been more or less commented and almost each piece of code should be self-explanatory. If you don't find it like that please **open an issue**.

I worked on this mainly alone so the code is far from perfect, super-modular, or anything like that. But together we can make it better. 

Thus, we welcome any contribution that you might have. Include coding style, comments, additions, or better strategies that you want to propose (of course after you have published your paper).
______________

### How to run the simulation

We're working on this piece of the documentation. Please bear with us while we upgrade the documents with better instructions.

A small tutorial can be found [here](https://github.com/eliabntt/GRADE-RR/blob/37ee985abccc6239bec7f22241c49da0acc5402c/OUR_CODE.md#main-code-tutorial-following-roughly-simulator_ros)

[Here](https://github.com/eliabntt/GRADE-RR/blob/37ee985abccc6239bec7f22241c49da0acc5402c/SAMPLES.md) you will learn what we already tried out, what we tested, what we used to run the simulations.

[Here](https://github.com/eliabntt/GRADE-RR/blob/37ee985abccc6239bec7f22241c49da0acc5402c/OUR_CODE.md) you can learn about our developed codebase, where you can find useful resources, and how you can edit them, file by file. 

### How to postprocess the data

Please check our dedicated repository [here](https://github.com/robot-perception-group/GRADE_tools).

### How to colorize the saved data

Simply run `python scripts/colorize.py --viewport_folder main_folder_with_npy_files`.
Check our code [here](https://github.com/eliabntt/GRADE-RR/blob/main/scripts/colorize.py), you can save images, images and videos, and decide which kind of data you want.

### How to get skeletal, vertices, and SMPL information while correcting bounding boxes

Look [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/smpl_and_bbox.py). This is mainly tuned for our data. However, it can be easily expanded to your own dataset.

### How to edit directly USD files

Check the tutorial [here](https://github.com/eliabntt/GRADE-RR/blob/37ee985abccc6239bec7f22241c49da0acc5402c/EDIT_USDS.md). This will help you convert USD to txt files for easy file processing.

### Isaac's edited files

Edited files are inside `isaac_internals`. The edited ones are the one that are copied by the `cp_local..` script. However, as per Isaac requirements, we had to include all the licenses and other files.

- _Shapenet_ minor edits regarding the main script since the dowload website seem down. We suggest to pre-download the dataset, unpack it, and set-up the environment folders as we show [here](https://github.com/eliabntt/GRADE-RR/blob/37ee985abccc6239bec7f22241c49da0acc5402c/OUR_CODE.md) to directly use the pre-downloaded data.
- _synthetic\_recorder_ created a custom extension to save our data, and offset the number of cameras. In that way we can save high-resolution images to the disk, while providing ROS smaller images. We found this faster than resizing images afterwards and caused less "issues".
- _synthetic\_utils_ we edited the `numpy.py` and the `syntheticdata.py` to save more data and have more flexibility. What is still missing (our bad) is the vertical fov of the camera, which is not directly exposed by Isaac Sim.
- In `setup_python_env.sh` we had to prevent the loading of `$SCRIPT_DIR/exts/omni.isaac.motion_planning/bin` (you can find it commented at the very end of line 8), to be able to run the system version of `move_base`. That module could be necessary for some of the Isaac extensions or configurations. Please be aware of this.

### How to move/control the camera/robot

You have several possibilities with and without ROS, with and without physics. Check them out [here](https://github.com/eliabntt/GRADE-RR/blob/37ee985abccc6239bec7f22241c49da0acc5402c/MOVEMENT.md)

### Possible missing textures/wrong paths

When loading humans or environments (or anything else) it may be necessar for you to edit the paths of the shaders, especially when moving between Windows and Linux.
To do that you can use the [`change_shader_path`](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/utils/misc_utils.py#L62) or the [correct paths](https://github.com/eliabntt/GRADE-RR/tree/main/scripts/process_paths) scripts.

Otherwise, you can simply process the text files as explained [here](https://github.com/eliabntt/GRADE-RR/blob/main/EDIT_USDS.md).

### Segmentation <-> instance

Instance segmentation files will save also the mappings between classes. An example on how to do the mapping and process those file is [here](https://github.com/robot-perception-group/GRADE-eval/blob/main/mapping_and_visualization/convert_classes.py).

_____
## Known issues
1. ros clock might have some delay in publishing. This implies that you need to sleep the simulation every time that component gets triggered. Other component behave consistently based on our tests. Alternatively, you can post-process the data as shown in [here](https://github.com/robot-perception-group/GRADE-eval)
2. BBOX3D are wrong for moving objects. The script [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/correct_data.py#L267) show a way to solve this.
3. Pose information is wrong for some moving objects. The code [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/correct_data.py#L224) will solve this.
4. Collisions for dynamic objects are not computed most of the times due to PhysX limitations. This is addressed by the new LiDAR-RTX of the new Isaac Sim version.
5. The rendering is not blocking. Multiple calls (especially for path tracing) are necessary. Thus, this usually disrupt the motion-vector data. A possible workaround is to do two rendering steps and save the motion-vector data, and then finish rendering to save the rgb information. See [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/replay_experiment.py#L390) an example on how to do that.
6. In the v2022 it is not possible to set indipendent vfov of the cameras
7. In the v2022 the internal PD control for the joints will NOT work using position setpoints. Also, the maximum velocity set is not considered.
8. In the v2022 the timeline gets updated automatically even if you do not want it. You need to keep track of the ctime and constantly re-update it to correctly generate the data you want.