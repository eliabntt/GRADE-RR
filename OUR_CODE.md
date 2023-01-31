General notices:
- we try to keep a general `dynamic\_things` list
- each Isaac command can be run with `omni.kit.commands.execute` ... etc
- the stage and context can be dynamically referenced with `omni.usd.get_context()[.get_stage()]`
- Each asset is loaded in a simulation into a "Prim". To get a prim one can usually run `stage.GetPrimAtPath(string_path)`
- The idea is to show you how the code can work. Clearly this has been adapted to our use-case, but by searching here you should have all the tools to build your stuff.
- For each object that you load you should call `clear_properties(path)`. This will ensure that the translate, rotate and scale operations are attached to the objects as expected.
- Each camera need to have a viewport attached. Each viewport is unique. The ROS cameras once triggered automatically occupy the first viewports (for whatever reason), thus first load all the ROS cameras and viewports and then the npy (if you want to differentiate).
- Some parameters might be useless for you, but all the codes use almost all of them. Be aware of that, you are free to edit this according to your own needs.

## USD and mesh files
In the corresponding folders you can find meshes and USD files of the robotino, the various drones used and the empty environments.

## Dowload Resources

You can download the zebra environment and other environments converted from SketchFab and Unreal Engine [here](https://keeper.mpdl.mpg.de/d/893ecd2a9a6b4c1587dc/). You can use them as a starting point. For the Savana environment you should download also the Zebras.

## Environment utils

Used to manage the environment.

With these functions you can load and center the environment, create a 2D occupancy map (only if the collisions are turned on), and center the environment.

This is where you want to act if you want to remove the centering of the environment, create a different kind of occupancy, or do something specific while loading.

Nothing super-fancy here.

## Human utils

Human management functions.

Can load the human, correct the paths of the assets if necessary, move it to the ground. Just showcasing some functions.

## Misc utils

Used as a collage library. 

There are tools to add semantic information, change the path of the textures, add colliders (or unset them), randomize lights and roughness of the materials, add translate and rotate animations, the service to position objects (that works with ROS), tools to rotate/translate objects, teleport the prim.

This is the main file you want to edit for example if you want to change the position strategy. Our position strategy uses FCL library from MoveIt and check collisios between two STL meshes. The system is done in a way in which it caches the environment and the robot stls at the beginning. We have different placement strategies for different objects (eg. humans, robot, and objects have different rules).

## Objects utils

Used to load the objects in the simulation. It will automatically convert the objects to the USD format to cache it. The objects are converted in local directories located in the GSO/shapenet folders. Semantic and collisions can be added to objects using thee utilities. Everything can be expanded easily by adding new object types.

The `shapenet` and `google_scanned_objects` folders are set up at runtime for example through the `os.environ["SHAPENET_LOCAL_DIR"]`.

## Robot utils

Various ways to create messages, add sensors, differentiate between poses (`create_diff_odom_message`), create viewports and publish stuff. Moreover, you want to use this to load your robot, set its initial joint locations and manage the trajectory. In general each component is loaded with auto publishing false and need to be automatically ticked or published. Some things like the odometry do not have a specific sensor but you can publish all the data that you want.

## Simulation utils

Mainly used for configuration settings (enalbe/disable extensions, change raytracing/pathtracing options), check that nucleus is powered up and ros is working, and manage the timeline.

## Custom data writing extension
You can setup the recorder as we will see shortly, give a `ros_cameras` offset (to write only a subset of viewports), and you can manually trigger it with `_update`.

## Main code tutorial (following roughly simulator_ros)
First step is to start the `kit`, load the base envirornment, and setup the simulation settings.

```python
CONFIG = {"display_options": 3286, "width": 1280, "height": 720, "headless": config["headless"].get()}
kit = SimulationApp(launch_config=CONFIG, experience=f"{os.environ['EXP_PATH']}/omni.isaac.sim.python.kit")
environment_setup()
environment = environment(config, rng, local_file_prefix)
```

Then you can (or you can skip) launch the ROS environments
```python
rospy.init_node("my_isaac_ros_app", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch_files = ros_launchers_setup(roslaunch, environment.env_limits_shifted, config)
parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files, force_log=True)
```

You can then load the base environment as a stage, create a SimulationContext and start the simulation
```python
omni.usd.get_context().open_stage(local_file_prefix + config["base_env_path"].get(), None)
simulation_context = SimulationContext(physics_dt=1.0 / config["physics_hz"].get(),
	                                       rendering_dt=1.0 / config["render_hz"].get(),
	                                       stage_units_in_meters=0.01)
simulation_context.start_simulation()
```

Add the ROS clock with `add_clock()`, which is necessary if you need to launch things from the code. You also need to "tick" the clock (i.e. publish it) and then call `parent.start()`.

Finally you should import the robot, set the initial position, add the ros components.

```
import_robot(robot_base_prim_path, n, local_file_prefix, base_robot_path)
move_robot(f"{robot_base_prim_path}{n}", [x / meters_per_unit, y / meters_per_unit, z / meters_per_unit], [roll, pitch, yaw], (environment.env_limits[5]) / meters_per_unit, config["is_iRotate"].get())
add_ros_components(robot_base_prim_path, n, ros_transform_components, ros_camera_list, viewport_window_list,
		                   camera_pose_frames, cam_pose_pubs, imus_handle_list, imu_pubs, robot_imu_frames,
		                   robot_odom_frames, odom_pubs,
		                   dynamic_prims, config, imu_sensor, imu_props, old_h_ap, old_v_ap)
```

You can decide if you want humans and objects. 
We first load all the humans, then check where we can place them and finally remove the ones that cannot be placed.

To load the objects simply call `google_ob_used, shapenet_ob_used = load_objects(config, environment, rng, dynamic_prims)`

Then we force the simulation to advance the clock only when we step the physics (and not the rendering) and to use the simulation time while setting the timeline to manually update.  
```
timeline.set_auto_update(False)
omni.kit.commands.execute("RosBridgeUseSimTime", use_sim_time=True)
omni.kit.commands.execute("RosBridgeUsePhysicsStepSimTime", use_physics_step_sim_time=True)
```

The main simulation loop then simply you can step the physics, count the iterations and publish when the ratio for the sensor is reached, and then render and publish the camera.

```python
timeline.forward/backward_one_frame()

# tick clock
omni.kit.commands.execute("RosBridgeTickComponent", path="/ROS_Clock")

# if ratio is met
c_pose, c_angle = pub_odom(robot_odom_frames, odom_pubs, _dc, meters_per_unit)

# rendering
pub_and_write_images(my_recorder, simulation_context, viewport_window_list, True, ros_camera_list, config["rtx_mode"].get())
```