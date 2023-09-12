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

Our code is thought in such a way that each robot is loaded pre-fixed with the `my_robot_` name, and this applies to each topic that is published from that robot. The exception lies in the `tf` topic, for which we will have a publisher for each robot. Data can be published in ROS and saved as npy files. If you want both, with the former using a lowres camera and the latter an high res camera you should first load all the robots, and then call `add_npy_cameras` adjusting the skipped camera of your `recorder`. See the [tips](https://github.com/eliabntt/GRADE-RR/blob/mainTipsAndTricks.md) readme for more insights.

## Your first code

[Here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/first_run.py) is a first example showing how to launch the simulation, load a basic environment, and perform some basic actions.

The workflow will always be the same. Import general modules, create the `SimulationApp`, import the IsaacSim related stuff, and proceed.

## Advanced scripts

Before adventuring here, please be sure to download our sample [world]() and [animated assets](). Those scripts will be incremental (i.e. based on the previous one). Please open all the downloaded USDs once at least to be sure that textures and everything else is correctly loaded.

We marked _Optional_ what can be skipped in future iterations of your code, but still, please go through them.

**Beore launching any simulation you need to start `roscore` if using ROS preferably with sim time set to true (`rosparam set use_sim_time true`)**

- Adding your own "world", and a robot [here](). The world can be either empty (thus you can skip loading), just with static objects, or with pre-placed animated objects (as in the zebra case).
- [Optional] Add some ROS components to the robot itself [here](). You can also create some custom "add all sensors" functions as we have done [here](https://github.com/eliabntt/GRADE-RR/blob/7d9cb9a3d75d57628adacb9b9f969909d7663f3d/simulator/utils/robot_utils.py#L557).
- [Optional] Add animated people, additional objects, and animate those [here]().
- [Optional] Launch your own SIL, either manually or from within your own simulation script [link]()
- Loop, [optional] publish ROS messages, and save data [here]().

## Additional scripts

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

## Known issues

1. ros clock might have some delay in publishing. This implies that you need to sleep the simulation every time that component gets triggered. Other component behave consistently based on our tests. Alternatively, you can post-process the data as shown in [here](https://github.com/robot-perception-group/GRADE-eval)
2. BBOX3D are wrong for moving objects. The script [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/correct_data.py#L267) show a way to solve this.
3. Pose information is wrong for some moving objects. The code [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/correct_data.py#L224) will solve this.
4. Collisions for dynamic objects are not computed most of the times due to PhysX limitations. This is addressed by the new LiDAR-RTX of the new Isaac Sim version.
5. The rendering is not blocking. Multiple calls (especially for path tracing) are necessary. Thus, this usually disrupt the motion-vector data. A possible workaround is to do two rendering steps and save the motion-vector data, and then finish rendering to save the rgb information. See [here](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/replay_experiment.py#L390) an example on how to do that.
6. In the v2022 it is not possible to set indipendent vfov of the cameras. It will take the hfov and use the AR to have a "correct" vfov.
7. In the v2022 the internal PD control for the joints will NOT work using position setpoints. Also, the maximum velocity set is not considered.
8. In the v2022 the timeline gets updated automatically even if you do not want it. You need to keep track of the ctime and constantly re-update it to correctly generate the data you want.