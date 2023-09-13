# Which parameters are available and how to edit them

Note: we try our best to update this consistently, but there might be some misalignment, this should be normal also considering different code runs. For example the "init_loc" parameter can be either a number or a list, depending on which version of the code you are running.

The main simulation parameters are the last ones in this page.
_____

### Python params for simulator/smpl_and_bbox

- `experiment_folder`, mandatory, the experiment folder with the USD file and the info file
- `body` When true process the bodies
- `garments` When true process the garments
- `base_path` Human prim base path, i.e. filtering prim paths
- `headless` Whether run this headless or not
- `write` Whether to write results
- `fast` Whether to write only the axis-aligned box or the oriented one, if False, the program will be slow
    - `both` Whether to write both vertex types -- preference in code is both 
- `only_exp` Whether to export only the experiment (considering the reverse strategy) or the whole sequences
- `get_skel` Whether to get/include the skeleton info
- `skel_root` This is a recognizable last part of the root of the skeleton prim, in our case _avg_root It will process ONLY the path of which the last part is this root

_____
### Python params for scripts/colorize

- `viewport_folder` mandatory, the "viewport" folder where npy data is saved
- `img_id` if negative process the whole sequence, otherwise just the img_id
- `save_imgs` if save images
- `save_video` if save videos (will produce two videos using ffmpeg)
- `always_update_map` if always updating the instance mapping. default to false using the first one. Useful if you toggle objects
- `semantics` if generate semantic
- `output_dir` output directory
_____
### Params for scripts/process_paths
- `config_file` configuration file with options to normalize the path, and critical keys that will be used in the main folder. You want to change these to your paths
- `input` input USDA file. Check the main readme on how to convert the USD file and convert back or use the .sh script
- `output_name` output USDA file
- `output_dir` output dir
_____

### Python params for the main simulator/[*simulation.py, multi_robot_sim.py] files
All the parameters are optional with the exception of the config file.
If you remove something from the config file please be sure that is not used in the code. Otherwise, it will crash. 

- `config_file` mandatory, the yaml file with mosst of the params
- `headless` if it's true the visualization is turned off
- `rtx_mode` if it's true the simulation will launch with the RTX rendering (faster), else the PathTracing is used
- `record` if it's true it will write to disk
- `debug_vis` if it's true it will loop visualization and ros camera publishing
- `neverending` if it's true it will continue forever the main loop
- `fix_env` you can set this to the _name_ of the sub-folder containing the environment that you want to load

### Simulation Config YAML params -- these are described for how they are used. You can easily edit and modiy their behavior (e.g. `human_path` can be your only human asset)

***"Mandatory" params***
- `env_path`: folder containing the subfolders of the environments (`env_path/env1, env_path/env2, ...`). You can randomly chose or use `fix_env` to specify an environment. 
- `human_path`: folder that contains the subfolders of the human animated assets, separated in the various datasets (`human_path/dataset1/animationX`,`human_path/dataset2/animationX`)
- `base_env_path`: global path of the basic environment (the background). NOTE: some configs cannot be changed from the code
- `usd_robot_path`: global path of the USD of the robot. Can be an array as for multi_robot case
- `robot_mesh_path`: global path of the mesh of the robot. Can be an array as for multi_robot case
- `out_folder`: path of the output folder in which we will save the ROS logs, and the map
- `out_folder_npy`: path of the output folder in which we will save the groundtruth from the simulator code (not the rosbags)
- `num_robots`: number of robots
- `_recorder_settings`: what to save or what not to save. Note that some things are not implemented. I strongly suggest to NOT save colorize data. Motion-vectors can be saved with the strategy shown in replay experiment
- `fps` the fps of the simulation
- `physics_hz`: NOTE THAT THIS IS THE RATE OF CLOCK AND IMU
- `render_hz`: LEAVE IT EQUAL TO PHYSICS HZ
- `env_prim_path` IsaacSim internal path for the prim of the environment
- `robot_base_prim_path` same thing for the robot (the number of the robot is the postfix)
- `is_iRotate` whether the robot is Robotino or not, this changes the launched ROS and some settings. Note that this can be a vector, and can be expanded to different robots

***THE OPTIONAL PARAMETER NEED TO BE EXPLICITLY ADDRESSED IN THE CODE. This wants to be a reference to search code and understand what is going on***

***Depending on usage params - experiment***
- `experiment_length`: camera frames length of the experiment (`seconds * camera_fps`), will be overridden by `neverending`. In the savana experiment this has not been used (experiment ends when last waypoint reached)
- `num_humans`: depends on your usage, can be fixed, minimum number or whatever you want
- `[robot,npy]_sensor_size` camera sensor size for robot and npy data. Can be equal. Npy not necessary if not loaded.
- `bootstrap_exploration`: seconds to boostrap the simulation before starting from time 0 (min(abs(this_value), 1/(physics_hz/ratio_camera)). It sets the time negative and cicle through physics and rendering.
- `reverse_strategy`: timeline reverse strategy based on the loaded animation lengths. Possibilities are [min, max, avg, half, none], works only with animated sequences. It makes the timeline going backward/forward based on this. It will roll back the simulation timeline (not the time, just the animation). This uses the animation length (see `paper_simulation.py`)
- `anim_exp_len` an alternative of `reverse_strategy`, rewinding the simulation after this many frames
- `clean_base_env` whether to remove some things from the base environment loaded at the beginning.
- `reload_references` whether to reload the references of the assets or not. Sometimes it might be necessary (seems solved in the newer versions)
- `generate_map` whether to generate the occupancy map or not. Works only if stls are loaded. Suggest to use only with limited environments (it takes a while to add collisions).

***Depending on usage params - robot movement***
- `autonomous`: true -> use FUEL, false -> use random goals (not fully tested), this is applicable only to the main paper simulation, used with autonomous=True. just to show it.
- `use_robot_traj`: Whether to use or not a predefined traj *not physics enabled*
- `use_joint_traj`: Whether to use or not joint trajecotry *physics enabled*. This cannot be true at the same time of robot_traj.
- `robot_traj`: The trajectory remember that movement will be linear and instantaneous. No acceleration or anything. This implies no odom, nor IMU data. If you want those, please add the same trajectory to a joint publisher. 
- `init_loc`: initial location for the robot (the elements can be vectors as in the multi-robot case)

***Depending on usage params - humans***
- `max_distance_human_ground`: max distance from human to ground to be consider to force the first frame grounding of animations
- `allow_collision`: max number of collisions allowed between the stl of the human and the stl of the environment
- `human_base_prim_path` for the humans (the number of the human is the postfix)
- `[max,min]_human_anim_len`: [Max,Min]imum human animation to be considered.

***Depending on usage params - objects***
- `obstacles`: increase those numbers to load shapenet or google objects (or any other objects)
- `google_obj_folder`: google_scanned_objects folder. Structure is `folder/exported_usd` and `folder/assets`
- `google_obj_shortlist`: shortlist some objects, not fully tested
- `shapenet_local_dir`: local dir of ShapeNet *suggestion is to download this beforehand*
- `shapenet_username`: if want to download on the fly. Last time I tried it was not working anymore.
- `shapenet_password`: if want to download on the fly. Last time I tried it was not working anymore.
- `synsetId`: shortlist some objects, not fully tested
- `modelId`: shortlist some objects, not fully tested

***Depending on usage params - simulation***
- `ratio_[tf,odom,camera,...]`: physics_hz/ratio_tf = tf publish hz
- `_random_light` 
    - `intensity` If intensity needs to be changed
    - `color` If color needs to be changed
    - `intensity_interval`
    - `during_experiment` Change color/intensity during the experiment
    - `n-frames` if during experiment True switch the color of the light
    - `smooth` NOT IMPLEMENTED
- `_random_roughness` Roughness/reflectance of the materials
    - `enabled` If enabled
    - `intensity_interval`

***Depending on usage params - others***
- `only_placement` if the placement strategy should be the only ROS thing launched. Following a similar strategy all ROS can be disabled.
- `use_stl` wheter to load the STLs of env/humans or not. This will have repercussion but gives the possibility to avoid generating/loading the STL files.