# HOW TO CONFIG IT

### Python params
- `config_file` mandatory, the yaml file with mosst of the params
- `headless` if it's true the visualization is turned off
- `rtx_mode` if it's true the simulation will launch with the RTX rendering (faster), else the PathTracing
- `record` if it's true it will write to disk
- `debug_vis` if it's true it will loop visualization and ros camera publishing
- `neverending` if it's true it will continue forever the main loop
- `fix_env` you can set this to the _name_ of the sub-folder containing the environment that you want to load

### YAML params

- `env_path`: folder containing the subfolders of the environments (`env_path/env1, env_path/env2, ...`)
- `human_path`: folder that contains the subfolders of the human animated assets, separated in the various datasets (`human_path/dataset1/animationX`,`human_path/dataset2/animationX`)
- `base_env_path`: global path of the basic environment (the background). NOTE: some configs cannot be changed from the code
- `base_robot_path`: global path of the USD of the robot
- `out_folder`: path of the output folder in which we will save the ROS logs, the map
- `out_folder_npy`: path of the output folder in which we will save the groundtruth from the simulator code (not the rosbags)
- `num_robots`: number of robots
- `num_humans`: min 7, if 0 change the main code, this is randomized between 7 and this number [see here](https://github.com/eliabntt/isaac_sim_manager/blob/main/simulator/simulator_ros.py#L386)
- `max_distance_human_ground`: max distance from human to ground to be consider to force the first frame grounding of animations
- `allow_collision`: max number of collisions allowed between the stl of the human and the stl of the environment
- `experiment_length`: camera frames length (`seconds * camera_fps`)
- `autonomous`: true -> use FUEL, false -> use random goals (not fully tested)
- `obstacles`: increase those numbers to load shapenet or google objects
- `physics_hz`: NOTE THAT THIS IS THE RATE OF CLOCK AND IMU
- `render_hz`: LEAVE IT EQUAL TO PHYSICS HZ
- `ratio_tf`: physics_hz/ratio_tf = tf publish hz
- `ratio_odom`: physics_hz/ratio_odom = odom publish hz
- `ratio_camera`: physics_hz/ratio_cam = imgs publish hz
- `bootstrap_exploration`: seconds to boostrap exploration (min(abs(this_value), 1/(physics_hz/ratio_camera))
- `reverse_strategy`: timeline reverse strategy based on the loaded animation lengths. Possibilities are [min, max, avg, half, none], works only with animated sequences. It makes the timeline going backward/forward based on this.
- `robot_sensor_size`
- `npy_sensor_size`
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
- `env_prim_path` IsaacSim internal path for the environment
- `robot_base_prim_path` for the robot (the number of the robot is the postfix)
- `human_base_prim_path` for the humans (the number of the human is the postfix)
- `max_human_anim_len`: Maximum human animation to be considered. Longer than this will be discarded
- `min_human_anim_len`: Minimum human animation to be considered. Shorter than this will be discarded
- `_recorder_settings`
- `google_obj_folder`: google_scanned_objects folder. Structure is `folder/exported_usd` and `folder/assets`
- `google_obj_shortlist`: ""
- `shapenet_local_dir`: local dir of ShapeNet *suggestion is to download this beforehand*
- `shapenet_username`: if want to download on the fly. Last time I tried it was not working anymore.
- `shapenet_password`: if want to download on the fly. Last time I tried it was not working anymore.
- `synsetId`: "random"
- `modelId`: "random"
### the following cannot be both true at the same time
### if so, only the robot traj will be executed
### if both false we assume an external source is publishing something to your robot (in our case on /my_robot_0/joint_commands)
- use_robot_traj: False # this is an absolute value. Note that the main root link and the actual position of the robot may differ based  on the initial shift(which remains constant)
- use_joint_traj: False # this is a relative value w.r.t. the starting location
- robot_traj: # remember that movement will be linear and instantaneous. No acceleration or anything. This implies no odom, nor IMU data. If you want those, please add the same trajectory to a joint publisher. 
