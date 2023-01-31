0. The current script suppose TWO workspaces `catkin_ws` with the main repo as setted up in the main branch and `irotate_ws` containing the ws for the repo at step 1. 

1. Download and install iRotate from [here](https://github.com/eliabntt/irotate_active_slam/tree/isaac). Remember to switch the branch to `isaac`.
	a. Note that this is NOT fully tested and NOT the paper version of the code. However, we tested it and it should work as expected.
	b. run `git submodule update --recursive` to download the correct version of the controller
2. Checkout the branch _irotate-edits_ in this repo
3. run the simulation ```./python.sh isaac_sim_manager/simulator/simulator_ros.py  --/renderer/enabled='rtx,iray'  --config="/your_global_path/irotate_config.yaml" --neverending=True --record=False --rtx_mode=True --headless=False --debug_vis=False```. REMEMBER to change the config file with your own absolute paths.
4. run `python[3] irotate_specific/republish_tf.py`
5. run `irotate` as explained in the repo. A set of commands could be:
```
roslaunch robotino_simulations world.launch
roslaunch robotino_simulations rtabmap.launch delete:=-d
roslaunch active_slam active_node.launch
roslaunch robotino_mpc robotino_mpc.launch
roslaunch robotino_fsm robotino_fsm.launch kind:=2 only_last_set:=false pre_fix:=true mid_optimizer:=true weighted_avg:=true robot_odom:=/odometry/filtered cam_odom:=/camera/odometry/filtered
```


Some notes:

This is just a sample of the capabilities of the system.

We let the robot start from `0,0,0` and `yaw=0`. If you change this, like with the previous work, you need to change the ekfs accordingly.

The transform `world->map` is constant. `map->odom` is done by `rtabmap`. `odom->base_link` is done from the ekfs. 

Isaac is setted up to publish the tfs to the `/tf2` topic. Step 4 is necessary to publish everything back to the `/tf` cleaned up of the ground truth estimation.

The custom joint controller has been updated. You need to be sure you are running the one from irotate repository.
Thus, we need either to build everything in the same workspace or use `source ... --extend` if you are using two workspaces. 
You can eventually change the scripts to have it working how you want.

You can launch an rviz visualization with `rviz -d irotate_specific irotate.rviz`