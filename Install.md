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

### Isaac's edited files

Edited files are inside `isaac_internals`. The edited ones are the one that are copied by the `cp_local..` script. However, as per Isaac requirements, we had to include all the licenses and other files.

- _Shapenet_ minor edits regarding the main script since the dowload website seem down. We suggest to pre-download the dataset, unpack it, and set-up the environment folders as we show [here](https://github.com/eliabntt/GRADE-RR/blob/main/OUR_CODE.md) to directly use the pre-downloaded data.
- _synthetic\_recorder_ created a custom extension to save our data, and offset the number of cameras. In that way we can save high-resolution images to the disk, while providing ROS smaller images. We found this faster than resizing images afterwards and caused less "issues".
- _synthetic\_utils_ we edited the `numpy.py` and the `syntheticdata.py` to save more data and have more flexibility. What is still missing (our bad) is the vertical fov of the camera, which is not directly exposed by Isaac Sim.
- In `setup_python_env.sh` we had to prevent the loading of `$SCRIPT_DIR/exts/omni.isaac.motion_planning/bin` (you can find it commented at the very end of line 8), to be able to run the system version of `move_base`. That module could be necessary for some of the Isaac extensions or configurations. Please be aware of this.