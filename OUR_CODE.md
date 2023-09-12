General notices:
- we try to keep a general `dynamic\_things` list
- each Isaac command can be run with `omni.kit.commands.execute` ... etc
- the stage and context can be dynamically referenced with `omni.usd.get_context()[.get_stage()]`
- Each asset is loaded in a simulation into a "Prim". To get a prim one can usually run `stage.GetPrimAtPath(string_path)`
- The idea is to show you how the code can work. Clearly this has been adapted to our use-case, but by searching here you should have all the tools to build your stuff.
- For each object that you load you should call `clear_properties(path)`. This will ensure that the translate, rotate and scale operations are attached to the objects as expected.
- Each camera need to have a viewport attached. Each viewport is unique. The ROS cameras once triggered automatically occupy the first viewports (for whatever reason), thus first load all the ROS cameras and viewports and then the npy (if you want to differentiate).
- Some parameters might be useless for you, but all the codes use almost all of them. Be aware of that, you are free to edit this according to your own needs.

## Dowload Resources

You can download the zebra environment and other environments converted from SketchFab and Unreal Engine [here](https://keeper.mpdl.mpg.de/d/893ecd2a9a6b4c1587dc/). You can use them as a starting point. For the Savana environment you should download also the Zebras.


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