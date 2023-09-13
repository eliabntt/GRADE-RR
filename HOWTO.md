## Requirements and basic software installation

Please check the [requirements](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/requirements.html) on the official page.

Then download the omniverse launcher and install Nucleus, Cache, and Isaac Sim.

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

## Main concept

Our programs all follow the same structure.
- load the basic kit and start the initial simulation
- load a basic environment with some settings pre-applied (some config changes cannot be made with the code itself)
- load libraries and settings
- load your main environment
- edit the environment
- load the robots
- attach sensors to the robot
- correct the camera fov (bug in Isaac that changes it)
- [optional] load and place humans, objects and animate objects
- setup information recorder
- loop the simulation and publish/write the information when necessary

Every aspect can be personalized or adapted. The basic environment could be your final one, the humans/animations can be present or placed in a different way, robot can have your set of sensors or your own publishing rate.

Our code is thought in such a way that each robot is loaded pre-fixed with the `my_robot_` name, and this applies to each topic that is published from that robot. The exception lies in the `tf` topic, for which we will have a publisher for each robot. Data can be published in ROS and saved as npy files. If you want both, with the former using a lowres camera and the latter an high res camera you should first load all the robots, and then call `add_npy_cameras` adjusting the skipped camera of your `recorder`. See the [tips](https://github.com/eliabntt/GRADE-RR/blob/main/TipsAndTricks.md) readme for more insights.

## Your first code

[Here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/first_run.py) is a first example showing how to launch the simulation, load a basic environment, and perform some basic actions.

The workflow will always be the same. Import general modules, create the `SimulationApp`, import the IsaacSim related stuff, and proceed. Please, look at the comments in the code directly. Brief explanations are also given below.

## Going past your first code

Before adventuring here, please be sure to download our sample [world]() and [animated assets](). Those scripts will be incremental (i.e. based on the previous one). Please open all the downloaded USDs once at least to be sure that textures and everything else is correctly loaded.

We marked _Optional_ what can be skipped in future iterations of _your_ code, but still, please go through them. They will go step by step from the general environment to the animated house.

**Beore launching any simulation that need ros you need to start `roscore` if using ROS preferably with sim time set to true (`rosparam set use_sim_time true`)**

In these codes, we consider our provided sampled world, the animated assets, and the drone provided with this repository. For the objects, you will find a note in the corresponding tutorial details. Additional samples (our used code, adapted from v2021), will be added in the next section.

##### Still WIP, need to add links and make sure that the code works. But most of it should work rn.

- Using a config file, adding your own "world", and a robot [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/world_and_robot.py). 
    <details closed>

    - To create a robot you can either import our `usds/drone_2022.usd` or `usds/robotino.usd`, use your own URDF [link](https://docs.omniverse.nvidia.com/isaacsim/latest/ext_omni_isaac_urdf.html), create your own USD (add a mesh and attach some joints to it, [link](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_gui_simple_robot.html)), or use one of the already available models. For now, the USD file is enough.
    - The world can be either empty (thus you can skip loading), just with static objects, or with pre-placed animated objects (as in the zebra case). The world needs to be placed into a subfolder, e.g. `worlds/Savana/...`. Inside, you could (not mandatory) have:
        - `npy` file with the limits of the environment
        - `stl` file with the 3D occupancy of the environment
    If you do NOT have those, just disable the flags in the config file (see last point of this list). Otherwise, they will be used as shown [here](https://github.com/eliabntt/GRADE-RR/blob/455891d5021009695a5da13c4feda0ceb258d476/simulator/utils/environment_utils.py).
    - You will also see how to add colliders to the environment, how to generate a 2D occupancy map, how to use the meters per unit, how to move the robot before starting the simulation (by moving the joints).
    - Launch this with `./python.sh simulator/world_and_robot.py --config="/your_full_path/simulator/world_and_robot.yaml" --fix_env=Something`. `--config` is mandatory, `--fix_env` will tell to the system to select the `Something` world from the `world` environments folder, e.g. `Sample_house`
    </details closed>

- [Optional] Fix the rendering engine, add and publish some ROS components to the robot itself [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/robot_with_ros.py).  
    <details closed>

    - You will see how to add the clock to the simulation. Thanks to how we define it [here](https://github.com/eliabntt/GRADE-RR/blob/455891d5021009695a5da13c4feda0ceb258d476/simulator/utils/robot_utils.py#L274) the clock will tick with pysics steps, but will need to be manually published.
    - Our phylosophy is to manually publish ROS messages for better flexibility
    - We will show both how to add single components, or a batch of them, i.e. through custom "add all sensors" functions as we have done [here](https://github.com/eliabntt/GRADE-RR/blob/7d9cb9a3d75d57628adacb9b9f969909d7663f3d/simulator/utils/robot_utils.py#L557).
    - How to publish data (either manually with ROS messages or using the internal Isaac Components)
    - You can then fix the rendering engine (path vs raytracing), and get to know the `sleeping` function
    - Place a breakpoint somewhere and try to manually render the environment while the timeline is playing (not using sleeping). Note how the rendering will advance the timeline of more than what you want. This does not affect the physics, but will affect the animations. Keep this in mind. See [here](https://github.com/eliabntt/GRADE-RR/blob/6e42652201509ed7ad95624d9a551e24fe5ce03c/TipsAndTricks.md#L38) for more details. 
    - Launch this with `./python.sh simulator/robot_with_ros.py --config="/your_full_path/simulator/robot_with_ros.yaml" --fix_env=Something`. `--config` is mandatory, `--fix_env` will tell to the system to select the `Something` world from the `world` environments folder, e.g. `Sample_house`
    </details closed>
- [Optional] Add animated people, additional objects, and animate those while solving the timeline problem [here](). 
    <details closed>

    - You can get a sample human from [here](). Soon, we will upload our collection. Since then, you can follow our other repository [here](https://github.com/eliabntt/animated_human_SMPL_to_USD) to convert your SMPL models to USD. The preferred folder structure is `main/dataset/ID`, you will provide the `main` folder to allow the randomizer to work.
    - You can either place the models manually into your world beforehand (see the zebra case), use pre-fixed (or random) locations, or use a placement technique. Our placement technique will be explored in the additional scripts since it requires setting up the catkin workspace as well.
    - For the objects please download at least some assets from ShapeNetv2 or GSO websites. If not, please comment out that part of the code, or adapt it to your own assets. We think the GSO part can be made general quite easily. Paths should be `../gso/folders_of_the_objects` and `../shapenet/synsetIds/...`. For ShapeNet please also add `../shapenet/v1_csv/all_of_the_synset_csvs`. In the config add the `gso` and the `shapenet` folders. Additional options are there.
    - The animation will use the timeline interface see [here](https://github.com/eliabntt/GRADE-RR/blob/064c1b888727c6faa191f88519184dc272a8b950/simulator/utils/objects_utils.py#L135).
    - The objects loading code is [here](https://github.com/eliabntt/GRADE-RR/blob/064c1b888727c6faa191f88519184dc272a8b950/simulator/utils/objects_utils.py), for both shapenet and google scanned objects. You can see how the conversion works [here](https://github.com/eliabntt/GRADE-RR/blob/064c1b888727c6faa191f88519184dc272a8b950/simulator/utils/objects_utils.py#L65). The system will automatically save the converted USD for backup and to avoid re-conversion.
     - Launch this with `./python.sh simulator/people_and_objects.py --config="/your_full_path/simulator/humans_and_objects.yaml" --fix_env=Something`. `--config` is mandatory, `--fix_env` will tell to the system to select the `Something` world from the `world` environments folder, e.g. `Sample_house`
    </details closed>

- [Optional] Launch your own SIL from within your own simulation script, add some randomization (e.g. lights, textures etc) and save GT data [link]()

## Additional scripts


### Correct data and smpl_and_bbox

[This](https://github.com/eliabntt/GRADE-RR/blob/7d9cb9a3d75d57628adacb9b9f969909d7663f3d/simulator/smpl_and_bbox.py) and [this](https://github.com/eliabntt/GRADE-RR/blob/7d9cb9a3d75d57628adacb9b9f969909d7663f3d/simulator/correct_data.py) show how to access low-level information of the meshes, how it is possible to correct the 3DBbox and pose incorrect information.

### Zebra data generation and Animation Sequences

[This](https://github.com/eliabntt/GRADE-RR/blob/7d9cb9a3d75d57628adacb9b9f969909d7663f3d/simulator/zebra_datagen.py) is the code that we used to generate the data for the Zebra paper. Unfortunately, we cannot share the USDs of the environments, whith the exception of the Savanna one, due to licensing limitations.
You can however explore how to access low level animation sequences [link](https://github.com/eliabntt/GRADE-RR/blob/455891d5021009695a5da13c4feda0ceb258d476/simulator/utils/zebra_utils.py#L136) and how we managed to generate our data for the [Synthetic Data-based Detection of Zebras in Drone Imagery paper](https://arxiv.org/abs/2305.00432). Run it with `./python.sh GRADE-RR/simulator/zebra_datagen.py --/renderer/enabled='rtx,iray'  --config='configs/config_zebra_datagen.yaml' --headless=False --fix_env=Savana`

### Replay experiment

[This](https://github.com/eliabntt/GRADE-RR/blob/7d9cb9a3d75d57628adacb9b9f969909d7663f3d/simulator/replay_experiment.py) is a very useful piece of code. You can use this to replay any previously recorded experiment, modify the robot (or the scene conditions) and record new data. You can replay the experiment in two modalities, namely using teleport or by physically interpolating the trajectory. Note that the latter is subject to some drift due to the interpolation of the data itself. 

<details closed>
Please run

```
./python.sh GRADE-RR/simulator/replay_experiment.py --experiment_folder FOLDER
```
to do so.

In our code we show how to create a new stereo camera, save previously unsaved data, save motion-vector, and create a LiDAR sensor.

You need some information to be able to repeat an experiment. Namely, the joint positions. We load those [from the rosbags](https://github.com/eliabntt/GRADE-RR/blob/7d9cb9a3d75d57628adacb9b9f969909d7663f3d/simulator/replay_experiment.py#L177), although you can access them from the GT pose arrays.
</details closed>

### Paper autonomous indoor exploration (humans, objects, SIL, active SLAM etc)

### Multi robot management

## Known issues

1. ros clock might have some delay in publishing. This implies that you need to sleep the simulation every time that component gets triggered. Other component behave consistently based on our tests. Alternatively, you can post-process the data as shown in [here](https://github.com/robot-perception-group/GRADE-eval)
2. BBOX3D are wrong for moving objects. The script [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/correct_data.py#L267) show a way to solve this.
3. Pose information is wrong for some moving objects. The code [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/correct_data.py#L224) will solve this.
4. Collisions for dynamic objects are not computed most of the times due to PhysX limitations. This is addressed by the new LiDAR-RTX of the new Isaac Sim version. However, its management is not intuitive.
5. The rendering is not blocking. Multiple calls (especially for path tracing) are necessary. Thus, this usually disrupt the motion-vector data. A possible workaround is to do two rendering steps and save the motion-vector data, and then finish rendering to save the rgb information. See [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/replay_experiment.py#L390) an example on how to do that. Note that a rendering call is done just after the clocking.
6. In the v2022 it is not possible to set indipendent vfov of the cameras. It will take the hfov and use the AR to have a "correct" vfov.
7. In the v2022 the internal PD control for the joints will NOT work using position setpoints. Also, the maximum velocity set is not considered.
8. In the v2022 the timeline gets updated automatically even if you do not want it. You need to keep track of the ctime and constantly re-update it to correctly generate the data you want.