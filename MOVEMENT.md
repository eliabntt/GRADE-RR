# Movement control

There are two main aspects: ROS and Physics.

You can control the movement with or without ROS and with or without Physics.

In general, including ROS implies having physics.

With "With ROS" we mean that ROS is involved in the input. Clearly, you can always publish ROS information from the simulation.

When using joints, the system will essentially always use physics in some way. Clearly, as in Gazebo, you can disable gravity, collisions etc to your convenience. 

An important thing to remember is that by being physics enabled implies that joints will be affected by the mass of the object to which those are attached. Clearly, even with teleport this might be true. In practice, not being physically enabled requires you to disable gravity of the object and collisions. To disable gravity, do so by changing the property (if exists) of the asset, similarly to what we do for the collision [here](https://github.com/eliabntt/GRADE-RR/blob/7d9cb9a3d75d57628adacb9b9f969909d7663f3d/simulator/utils/misc_utils.py#L106).  With the exception of options 2 and 4 (which are pretty much equivalent) of the _Without ROS_ case. 

### With ROS
1. Attach a joint publisher/listener to the robot ([link](https://github.com/eliabntt/GRADE-RR/blob/7d9cb9a3d75d57628adacb9b9f969909d7663f3d/simulator/utils/robot_utils.py#L233)) and directly publish a `joint_commands` ROS message either on your own [link](https://docs.ros.org/en/melodic/api/control_msgs/html/msg/JointJog.html), using our 6DOF joint controller [link](https://github.com/eliabntt/custom_6dof_joint_controller), through MoveIt (see the [tutorial](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_ros_moveit.html)).
2. Use an embedded controller provided by IsaacSim and publish `cmd_vel` commands dependending on your use case.
3. Use ROS to publish setpoints in some ways, listen to the topic within the simulation loop, and fall back to the "without ROS" section.

### Without ROS 
1. Move the robot by sending _joint_ position/velocity setpoints directly from IsaacSim. This will output physics and will abide the settings that you use for your joints (mass, force...). The implementations within IsaacSim is through a PD control. An example of this has been implemented [here](https://github.com/eliabntt/GRADE-RR/blob/7d9cb9a3d75d57628adacb9b9f969909d7663f3d/simulator/utils/robot_utils.py#L760). Although the implementation there is different (it assume a set of predefined checkpoints) the concept is the same. To learn about stiffness and damping, please check [this](https://forums.developer.nvidia.com/t/stiffness-damping-setting-in-joint-drive/213916) or [this](https://docs.omniverse.nvidia.com/isaacsim/latest/ext_omni_isaac_motion_generation.html). Note that this has some issues in the 2022.2.1 version of the engine. Clearly, using this you can write your own controller. 

<details closed>

```python
import omni.kit.commands
from pxr import Sdf

omni.kit.commands.execute('ChangeProperty',
	prop_path=Sdf.Path('/World/robot/.../PrismaticJoint.drive:linear:physics:targetPosition'),
	value=10,
	prev=0)


```
```python
import omni.kit.commands
from pxr import Sdf

omni.kit.commands.execute('ChangeProperty',
	prop_path=Sdf.Path('/World/Cone/PrismaticJoint.drive:linear:physics:targetVelocity'),
	value=10,
	prev=0.0)
```
</details closed>

2. Use a strategy like the one we use for the [flying objects](https://github.com/eliabntt/GRADE-RR/blob/7d9cb9a3d75d57628adacb9b9f969909d7663f3d/simulator/utils/objects_utils.py#L191) adding [translation](https://github.com/eliabntt/GRADE-RR/blob/7d9cb9a3d75d57628adacb9b9f969909d7663f3d/simulator/utils/misc_utils.py#L288) and [rotation](https://github.com/eliabntt/GRADE-RR/blob/7d9cb9a3d75d57628adacb9b9f969909d7663f3d/simulator/utils/misc_utils.py#304) animations (also scale is possible). However, this does NOT include physics, collision or anything similar whatsoever. In this case the trajectory is followed blindly and interpolated based on your settings. 

3. Use [teleporting](https://github.com/eliabntt/GRADE-RR/blob/7d9cb9a3d75d57628adacb9b9f969909d7663f3d/simulator/utils/misc_utils.py#L497). For this see [replay experiment](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/replay_experiment.py) code. Note that running the physics steps will imply that the robot will be affected by physics (e.g. collisions, gravity etc)

4. Create a spline, animation sequence, or whatever and saving that to the USD file itself. Once loaded, the robot will behave as an animated object. Again, this won't follow physics low. It will still be affected by physics (accel, velocities) but not to collisions, gravity, etc. See [here](https://docs.omniverse.nvidia.com/extensions/latest/ext_animation-timeline.html) and related resources. Similar to #2.

5. Directly set joint status as done in [replay experiment](https://github.com/eliabntt/GRADE-RR/blob/7d9cb9a3d75d57628adacb9b9f969909d7663f3d/simulator/replay_experiment.py#L348) from within the simulation itself, although this is quite similar to do #1. 