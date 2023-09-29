import carb
import rospy
from omni.isaac.kit import SimulationApp
import argparse
import os
import time
import numpy as np
import roslaunch
from time import sleep
import yaml
import confuse

import ipdb, traceback, sys

def boolean_string(s):
    if s.lower() not in {'false', 'true'}:
        raise ValueError('Not a valid boolean string')
    return s.lower() == 'true'


try:
    parser = argparse.ArgumentParser(description="Dynamic Worlds Simulator")
    parser.add_argument("--config_file", type=str, default="config.yaml")
    parser.add_argument("--headless", type=boolean_string, default=True, help="Wheter to run it in headless mode or not")
    parser.add_argument("--rtx_mode", type=boolean_string, default=False, help="Use rtx when True, use path tracing when False")
    parser.add_argument("--record", type=boolean_string, default=True, help="Writing data to the disk")
    parser.add_argument("--debug_vis", type=boolean_string, default=False, help="When true continuosly loop the rendering")
    parser.add_argument("--neverending", type=boolean_string, default=False, help="Never stop the main loop")
    parser.add_argument("--fix_env", type=str, default="",
                        help="leave it empty to have a random env, fix it to use a fixed one. Useful for loop processing")

    args, unknown = parser.parse_known_args()
    config = confuse.Configuration("DynamicWorlds", __name__)
    config.set_file(args.config_file)
    config.set_args(args)
    can_start = True

    CONFIG = {"display_options": 3286, "width": 1280, "height": 720, "headless": config["headless"].get()}
    kit = SimulationApp(launch_config=CONFIG, experience=f"{os.environ['EXP_PATH']}/omni.isaac.sim.python.kit")

    # Cannot move before SimApp is launched
    import utils.misc_utils
    from utils.misc_utils import *
    from utils.robot_utils import *
    from utils.simulation_utils import *
    from utils.environment_utils import *


    simulation_environment_setup()
    # set timeline of the experiment
    timeline = setup_timeline(config)

    rospy.init_node("my_isaac_ros_app", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
    starting_pub = rospy.Publisher('starting_experiment', String)

    rng = np.random.default_rng()
    rng_state = np.random.get_state()

    local_file_prefix = ""

    # setup environment variables
    meters_per_unit = config["meters_per_unit"].get()
    environment = environment(config, rng, local_file_prefix, meters_per_unit)

    out_dir = os.path.join(config['out_folder'].get(), environment.env_name)
    out_dir_npy = os.path.join(config['out_folder_npy'].get(), environment.env_name)
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)

    os.environ["ROS_LOG_DIR"] = out_dir

    omni.usd.get_context().open_stage(local_file_prefix + config["base_env_path"].get(), None)

    # Wait two frames so that stage starts loading
    kit.update()
    kit.update()

    print("Loading stage...")
    while is_stage_loading():
        kit.update()
    print("Loading Complete")

    context = omni.usd.get_context()
    stage = context.get_stage()
    set_stage_up_axis("Z")
    if config["clean_base_env"].get():
        omni.kit.commands.execute("DeletePrimsCommand", paths=["/World/GroundPlane"])

    # do this AFTER loading the world
    simulation_context = SimulationContext(physics_dt=1.0 / config["physics_hz"].get(),
                                           rendering_dt=1.0 / config["render_hz"].get(),
                                           stage_units_in_meters=0.01)
    simulation_context.initialize_physics()
    physx_interface = omni.physx.acquire_physx_interface()
    physx_interface.start_simulation()

    _clock_graph = add_clock()  # add ROS clock

    simulation_context.play()
    for _ in range(10):
        simulation_context.step()
        og.Controller.evaluate_sync(_clock_graph)
        last_pub_time = rospy.Time.now()
    simulation_context.stop()

    kit.update()

    # use rtx while setting up!
    set_raytracing_settings(config["physics_hz"].get())
    env_prim_path = environment.load_and_center(config["env_prim_path"].get())
    process_semantics(config["env_prim_path"].get())

    ros_camera_list = []
    ros_transform_components = []  # list of tf and joint components, one (of each) for each robot
    viewport_window_list = []
    dynamic_prims = []
    imus_handle_list = []
    robot_odom_frames = []
    robot_imu_frames = []
    camera_pose_frames = []
    imu_pubs = []
    odom_pubs = []
    cam_pose_pubs = []

    simulation_context.play()
    for _ in range(100):
        og.Controller.evaluate_sync(_clock_graph)
        simulation_context.step()

    print("Loading robots..")

    from omni.isaac.sensor import _sensor
    _is = _sensor.acquire_imu_sensor_interface()

    _dc = dynamic_control_interface()

    robot_base_prim_path = config["robot_base_prim_path"].get()
    usd_robot_path = str(config["usd_robot_path"].get())
    old_h_ap = []
    old_v_ap = []
    robot_init_loc = []
    robot_init_ang = []

    simulation_context.stop()
    for n in range(config["num_robots"].get()):
        import_robot(robot_base_prim_path, n, usd_robot_path, local_file_prefix)
        if config["init_loc"].get()["use"]:
            # assuming we go here
            x = config["init_loc"].get()["x"][n]
            y = config["init_loc"].get()["y"][n]
            z = config["init_loc"].get()["z"][n]
            yaw = np.deg2rad(config["init_loc"].get()["yaw"][n])
            roll = np.deg2rad(config["init_loc"].get()["roll"][n])
            pitch = np.deg2rad(config["init_loc"].get()["pitch"][n])
        robot_init_loc.append([x,y,z])
        robot_init_ang.append([roll, pitch, yaw])

        set_drone_joints_init_loc(f"{robot_base_prim_path}{n}", [x / meters_per_unit, y / meters_per_unit, z / meters_per_unit], [roll, pitch, yaw],
                   (environment.env_limits[5]) / meters_per_unit)

        add_ros_components(robot_base_prim_path, n, ros_transform_components, ros_camera_list, viewport_window_list,
                           camera_pose_frames, cam_pose_pubs, imu_pubs, robot_imu_frames,
                           robot_odom_frames, odom_pubs, None, #lidars = None
                           dynamic_prims, config, old_h_ap, old_v_ap, _is, simulation_context, _clock_graph)

        kit.update()
        if config["use_robot_traj"].get():
            add_robot_traj(f"{robot_base_prim_path}{n}",config,meters_per_unit,timeline.get_time_codes_per_seconds())

    for n in range(config["num_robots"].get()):
        add_npy_viewport(viewport_window_list, robot_base_prim_path, n, old_h_ap, old_v_ap, config, simulation_context,
                         config["num_robots"].get())

    for _ in range(50):
        simulation_context.render()
    print("Loading robot complete")

    print("WARNING: CAMERA APERTURE MANUAL SET NO LONGER WORKS, NEEDS TO BE FIXED BY NVIDIA!!!!")
    time.sleep(5)
    for index, cam in enumerate(viewport_window_list):
        camera = stage.GetPrimAtPath(cam.get_active_camera())
        camera.GetAttribute("horizontalAperture").Set(old_h_ap[index])
        camera.GetAttribute("verticalAperture").Set(old_v_ap[index])

    # IT IS OF CRUCIAL IMPORTANCE THAT AFTER THIS POINT THE RENDER GETS DONE WITH THE SLEEPING CALL! OTHERWISE PATH TRACING SPP WILL GET RUINED
    if (config["rtx_mode"].get()):
        set_raytracing_settings(config["physics_hz"].get())
    else:
        set_pathtracing_settings(config["physics_hz"].get())

    omni.usd.get_context().get_selection().set_selected_prim_paths([], False)

    simulation_context.stop()
    simulation_context.play()
    for _ in range(5):
        simulation_context.step(render=False)
        sleeping(simulation_context, viewport_window_list, config["rtx_mode"].get())
    timeline.set_current_time(0)
    simulation_step = 0  # this is NOT the frame, this is the "step" (related to physics_hz)

    my_recorder = recorder_setup(config['_recorder_settings'].get(), out_dir_npy, config['record'].get())

    timeline.set_current_time(0)  # set to 0 to be sure that the first frame is recorded
    timeline.set_auto_update(False)

    # two times, this will ensure that totalSpp is reached
    sleeping(simulation_context, viewport_window_list, config["rtx_mode"].get())
    sleeping(simulation_context, viewport_window_list, config["rtx_mode"].get())

    if config['debug_vis'].get():
        cnt = 0
        while 1:
            cnt += 1
            if cnt % 10000 == 0:
                import ipdb

                ipdb.set_trace()
            print("DEBUGGING VIS")
            simulation_context.step(render=False)
            simulation_context.step(render=True)
            sleeping(simulation_context, viewport_window_list, config["rtx_mode"].get())
            for i, cam in enumerate(ros_camera_list):
                omni.kit.commands.execute("RosBridgeTickComponent", path=str(cam.GetPath()))

    ratio_camera = config["ratio_camera"].get()
    ratio_odom = config["ratio_odom"].get()
    ratio_tf = config["ratio_tf"].get()
    starting_to_pub = False
    my_recorder._enable_record = False
    forward = True
    goal_list = []
    exp_len = config["anim_exp_len"].get()
    if not config["use_robot_traj"].get() and config["use_joint_traj"].get():
        for elem in config["robot_traj"].get():
            goal_list.append([elem["pose"]["x"], elem["pose"]["y"], elem["pose"]["z"],
                              elem["pose"]["roll"], elem["pose"]["pitch"], elem["pose"]["yaw"]])

    while kit.is_running():
        if can_start:
            if config['record'].get():
                # reload_references("/World/home")
                sleeping(simulation_context, viewport_window_list, config["rtx_mode"].get())
                my_recorder._update()
                sleeping(simulation_context, viewport_window_list, config["rtx_mode"].get())
            starting_to_pub = True
            timeline.set_current_time(min(- 1 / (config["physics_hz"].get() / ratio_camera),
                                          -abs(config["bootstrap_exploration"].get())))
            simulation_step = int(timeline.get_current_time() * config["physics_hz"].get()) - 1
            print("Bootstrap started")
            can_start = False
        simulation_step += 1
        if starting_to_pub and simulation_step == 0:
            print("Starting recording NOW!")
            msg = String("starting")
            starting_pub.publish(msg)
            time.sleep(0.5)
            starting_to_pub = False
            if config['record'].get():
                my_recorder._enable_record = True

        # step the physics
        simulation_context.step(render=False)

        # get the current time in ROS
        print("Clocking...")
        og.Controller.evaluate_sync(_clock_graph)
        ctime = timeline.get_current_time()
        simulation_context.render()
        timeline.set_current_time(ctime)

        # publish IMU
        print("Publishing IMU...")
        pub_imu(_is, imu_pubs, robot_imu_frames, meters_per_unit)

        # publish joint status (ca 120 Hz)
        if simulation_step % ratio_tf == 0:
            print("Publishing joint/tf status...")
            for component in ros_transform_components:
                og.Controller.set(og.Controller.attribute(f"{component}/OnImpulseEvent.state:enableImpulse"), True)

        # publish odometry (60 hz)
        if simulation_step % ratio_odom == 0:
            print("Publishing odometry...")
            c_pose, c_angle = pub_odom(robot_odom_frames, odom_pubs, _dc, meters_per_unit)
            pub_cam_pose(camera_pose_frames, cam_pose_pubs, _dc, meters_per_unit)

        if config["use_joint_traj"].get():
            if len(goal_list)>0 and simulation_step >= 0:
                # this needs to be expanded to multiple robots
                goal_list = check_pose_and_goals(robot_init_loc[0], robot_init_ang[0], c_pose[0], c_angle[0], "/my_robot_0", goal_list, meters_per_unit, simulation_step == 0)
                if len(goal_list)==0:
                    break

        # we consider ratio_camera to forward the animation.
        # If you want it different ratio_animation < ratio_camera to avoid
        # two frames with the same animation point
        if simulation_step % ratio_camera == 0:
            if my_recorder._enable_record:
                # update the image counter externally so that we can use it in the recorder and all images have the same index
                my_recorder._counter += 1
            if (simulation_step > 0 and (simulation_step / ratio_camera + 1) % exp_len == 0):
                forward = not forward
            if  (timeline.get_current_time() - 1 / timeline.get_time_codes_per_seconds()<0):
                forward = True
            if forward:
                timeline.forward_one_frame()
            else:
                timeline.rewind_one_frame()

        # publish camera (30 hz)
        if simulation_step % ratio_camera == 0:
            ctime = timeline.get_current_time()
            print("Publishing cameras...")

            pub_and_write_images(simulation_context, viewport_window_list,
                                 ros_camera_list, config["rtx_mode"].get(), my_recorder)
            timeline.set_current_time(ctime)

except:
    extype, value, tb = sys.exc_info()
    traceback.print_exc()
    # ipdb.post_mortem(tb)
finally:
    for pub in odom_pubs:
        pub.unregister()
    for pub in imu_pubs:
        pub.unregister()
    for pub in cam_pose_pubs:
        pub.unregister()
    parent.shutdown()
    rospy.signal_shutdown("my_simulation complete")
    simulation_context.stop()
    try:
        kit.close()
    except:
        pass
