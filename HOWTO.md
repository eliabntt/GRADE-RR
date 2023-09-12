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

Independently on where you cloned the repository you need to run
`sh cp_local_to_different_folder.sh $CLONE_FOLDER $ISAAC_FOLDER`

This will copy the edited files from $1 (source) to the $2 (destination). You can use it in reverse (from Isaac to repo), or with any couple of folders.


## Misc

A general note: every script has been more or less commented and almost each piece of code should be self-explanatory. If you don't find it like that please **open an issue**.

I worked on this mainly alone so the code is far from perfect, super-modular, or anything like that. But together we can make it better. 

Thus, we welcome any contribution that you might have. Include coding style, comments, additions, or better strategies that you want to propose (of course after you have published your paper).


## How to start the simulation

To launch Isaac you can run `./isaac-sim.sh` from the main installation folder to launch the simulator. It is suggested to do this once before starting any other coding activity.

To control the simulation with your own code the general process is `./python.sh python_script args`. In `args` you can specify either python arguments arguments for the Isaac simulator itself (e.g. ` --/renderer/enabled='iray'`).

The functions that we use in our scripts are all contained in the `simulator/utils` folder. An explanation of each one of the function is given in their comment, while a brief overview is given [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/utils/UTILS.md).

## Your first code

[Here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/first_run.py) is a first example showing how to launch the simulation, load a basic environment, and perform some basic actions.

The workflow will always be the same. Import general modules, create the `SimulationApp`, import the IsaacSim related stuff, and proceed.

## Advanced scripts

A small tutorial can be found [here](https://github.com/eliabntt/GRADE-RR/blob/37ee985abccc6239bec7f22241c49da0acc5402c/OUR_CODE.md#main-code-tutorial-following-roughly-simulator_ros)

[Here](https://github.com/eliabntt/GRADE-RR/blob/37ee985abccc6239bec7f22241c49da0acc5402c/SAMPLES.md) you will learn what we already tried out, what we tested, what we used to run the simulations.

[Here](https://github.com/eliabntt/GRADE-RR/blob/37ee985abccc6239bec7f22241c49da0acc5402c/OUR_CODE.md) you can learn about our developed codebase, where you can find useful resources, and how you can edit them, file by file. 


## Tips and tricks

### Viewports vs Cameras
Each rendering component will have a viewport associated to that.

In our testing we found out that switching cameras related to a viewport (to reduce memory consumption) may lead to memory leakage and other problems.

What we suggest is to have one viewport for every thing that you want to render, be that a full-res camera or a ROS-camera.

If you want both high-res and low-res images, we suggest to have two viewports. It will slow down things (as rendering will be slower), but it will be easier to manage.


### Time

For robotics applications the time is goverened by the physics time. On the other hand, the default step of the simulation in Isaac is governed by the rendering. 

The time of the animations is goverened by the timeline. You can access that using
```
timeline = setup_timeline(config) # config containing some additional options
# or
timeline = omni.timeline.get_timeline_interface()
```

And perform various operations such as
```
timeline.set_current_time(0)
timeline.get_current_time()
timeline.forward/backward_one_frame()
```

`timeline.set_auto_update(False)` is used to stop the timeline advancing every rendering call. However, this is apparently not working in the current version of Isaac Sim. Thus, in the `sleeping()` function we constantly reset the time to the current time so that the rendering is correct.

### Rendering, manual interaction, and UI

The simulation app UI will be refreshed ONLY when you do a rendering call. 

For stepping the physics and rendering you have different options:
1. `kit.update()` will step both the physics and the rendering
2. `simulation_context.render()` will do only a SINGLE rendering step
3. `simulation_context.step()` will do both a rendering and physics step. Not always working
4. `simulation_context.step(render=False)` will do a physics step

My suggestion is to always work with a combination of `simulation_context.render()/step(render=False)` and to stick to that.

If needed, you will be able to interact with the application only when fast enough rendering calls are made. Sometimes, it is necessary to also step the physics to see the effects of your actions. A quick way to do this is to:
1. enter debug mode in python
2. run a loop such as
    ```
    for _ in range(1000):
        simulation_context.step(render=False)
        simulation_context.render()
    ```

#### *The rendering calls are NOT blocking. This means that every time you render it will do that for either 1) a fixed amount of time in case of RTX rendering, or 2) a single step for path tracing rendering. This has been solved by us through the `sleeping` function in the `simulation_utils.py`.*

### Save the GT information

The process is to either save stuff from the main simulation loop, or to use the synthetic recorder extension.

In the latter case you can use directly what we provide in `isaac_internals/exts/omni.isaac.synthetic_recorder/omni/isaac/synthetic_recorder/extension_custom.py` and expand it alongside with `isaac_internals/exts/omni.isaac.synthetic_utils/omni/isaac/synthetic_utils/writers/numpy.py` and `isaac_internals/exts/omni.isaac.synthetic_utils/omni/isaac/synthetic_utils/syntheticdata.py` code. 

Then you can create a recorder directly in your code using:

```
from omni.isaac.synthetic_recorder import extension_custom

my_recorder = extension_custom.MyRecorder()
my_recorder.on_startup() # necessary call

_settings = my_recorder.get_default_settings()
_settings["rgb"]["enabled"] = True # inspect and extend this dictionary

my_recorder.set_single_settings(_settings)
my_recorder._dir_name = os.path.join(out_path)
my_recorder._enable_record = True # set to false to disable
my_recorder.skip_cameras = 0 # number of viewports to skip

# do stuff

my_recorder._update() # write data if enabled
```

This will create the desired data for EACH viewport.

A shorter version is by using 
```
recorder = recorder_setup(recorder_setup(_recorder_settings, out_path, enabled, skip_cameras)
recorder._update()
```

All data can be also accessed in the main simulation loop. Some examples are the vertices, or the lidar information (see the replay experiment script).

Potentially, you could also get as output of the recorder `_update()` call all the information, edit, and publish them as ROS messages.

### Save the motion vector
This is not possible during rendering itself. To save it you need to manually render twice and then save the motion vector itself. See the repeating experiment tool for an example on how to do that. Indeed, the motion vector can be only visualized by the default Isaac installation and not saved (see [here](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.sensor/docs/index.html#module-omni.isaac.sensor.scripts.camera))

### Postprocess the data

Please check our dedicated repository [here](https://github.com/robot-perception-group/GRADE_tools).

### Colorize the saved data

Simply run `python scripts/colorize.py --viewport_folder main_folder_with_npy_files`.
Check our code [here](https://github.com/eliabntt/GRADE-RR/blob/main/scripts/colorize.py), you can save images, images and videos, and decide which kind of data you want.

### How to get skeletal, vertices, and SMPL information while correcting bounding boxes

Look [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/smpl_and_bbox.py). This is mainly tuned for our data. However, it can be easily expanded to your own dataset.

### How to edit directly USD files

Check the tutorial [here](https://github.com/eliabntt/GRADE-RR/blob/37ee985abccc6239bec7f22241c49da0acc5402c/EDIT_USDS.md). This will help you convert USD to txt files for easy file processing.

### Shapenet, Google Scanned Objects

_ShapeNet_ We suggest to pre-download the dataset, unpack it, and set-up the environment folders as we show [here](https://github.com/eliabntt/GRADE-RR/blob/37ee985abccc6239bec7f22241c49da0acc5402c/OUR_CODE.md) to directly use the pre-downloaded data.

_GoogleScannedObjects_ Again, please download the dataset beforehand.

### How to move/control the camera/robot

You have several possibilities with and without ROS, with and without physics. Check them out [here](https://github.com/eliabntt/GRADE-RR/blob/37ee985abccc6239bec7f22241c49da0acc5402c/MOVEMENT.md)

### Possible missing textures/wrong paths

When loading humans or environments (or anything else) it may be necessar for you to edit the paths of the shaders, especially when moving between Windows and Linux.
To do that you can use the [`change_shader_path`](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/utils/misc_utils.py#L62) or the [correct paths](https://github.com/eliabntt/GRADE-RR/tree/main/scripts/process_paths) scripts.

Otherwise, you can simply process the text files as explained [here](https://github.com/eliabntt/GRADE-RR/blob/main/EDIT_USDS.md).

### Segmentation <-> instance

Instance segmentation files will save also the mappings between classes. An example on how to do the mapping and process those file is [here](https://github.com/robot-perception-group/GRADE-eval/blob/main/mapping_and_visualization/convert_classes.py).


## Known issues

1. ros clock might have some delay in publishing. This implies that you need to sleep the simulation every time that component gets triggered. Other component behave consistently based on our tests. Alternatively, you can post-process the data as shown in [here](https://github.com/robot-perception-group/GRADE-eval)
2. BBOX3D are wrong for moving objects. The script [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/correct_data.py#L267) show a way to solve this.
3. Pose information is wrong for some moving objects. The code [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/correct_data.py#L224) will solve this.
4. Collisions for dynamic objects are not computed most of the times due to PhysX limitations. This is addressed by the new LiDAR-RTX of the new Isaac Sim version.
5. The rendering is not blocking. Multiple calls (especially for path tracing) are necessary. Thus, this usually disrupt the motion-vector data. A possible workaround is to do two rendering steps and save the motion-vector data, and then finish rendering to save the rgb information. See [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/replay_experiment.py#L390) an example on how to do that.
6. In the v2022 it is not possible to set indipendent vfov of the cameras. It will take the hfov and use the AR to have a "correct" vfov.
7. In the v2022 the internal PD control for the joints will NOT work using position setpoints. Also, the maximum velocity set is not considered.
8. In the v2022 the timeline gets updated automatically even if you do not want it. You need to keep track of the ctime and constantly re-update it to correctly generate the data you want.