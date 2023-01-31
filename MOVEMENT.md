# Movement control

There are two main aspects: ROS and Physics.

You can control the movement with or without ROS and with or without Physics.

In general, including ROS implies having physics.

With "With ROS" we mean that ROS is involved in the input. Clearly, you can always publish ROS information from the simulation.

With ROS:
1. Directly publish `joint_commands`. This is doable even directly within the main simulation loop as it is setted up to already be a node. This is what we did in our `paper_simulation` and `iRotate` code.

2. Use an embedded controller provided by IssacSim and publish `cmd_vel` or `moveit` commands dependending on your use case.

3. Use ROS to publish setpoints and fall back to the "without ROS" section.

Without ROS:

1. Move the robot by sending joint position/velocity setpoints directly from IsaacSim. An example of this has been implemented in `savana_simulation`. This will output physics and will abide the settings that you use for your joints (mass, force...). *WITH PHYSICS*

2. Use a strategy like the one we use for the [flying objects](https://github.com/eliabntt/isaac_sim_manager/blob/main/simulator/objects_utils.py#L166). However, this does NOT include physics, collision or anything similar whatsoever. In this case the trajectory is followed blindly and interpolated based on your settings. *WITHOUT PHYSICS*

3. Use teleporting. For this see [replay experiment](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/replay_experiment.py) code. *WITHOUT PHYSICS*

4. Create the environment beforehand and set keypoints (essentially manually doing 2.) *WITHOUT PHYSICS*

5. Directly set joint status as done in [replay experiment](https://github.com/eliabntt/GRADE-RR/blob/main/simulator/replay_experiment.py) from within the simulation itself. *WITH PHYSICS*

_______
## More details and examples

An important thing to remember is that by being physics enabled implies that joints will be affected by the mass of the object to which those are attached. 
The comprehension is easier than Gazebo in this case. Please test it manually before loading the robot in your simulation.

An important aspect, that we missed, is the mass of the robot. The robot may behave correctly while having an incorrect mass. This implies that acceleration might be off (physically plausible, but not what you want).

The robot can be either created manually or using the available URDF importer.

### Autonomous exploration (paper version)

By using our paper code you will experience our full system. 
The trajectory will be created by FUEL, passed to a NMPC, which will compute a plausible robot trajectory, then to our customized 6DOF joint PID controller, and published as joint commands. You find all the packages and the descriptions [here](https://github.com/eliabntt/ros_isaac_drone).

Once the simulation is launched you do not need to do anything at all. 

### Autonomous exploration (iRotate)
We slightly edited the iRotate package to conform to our topic structure. Basically every robot publish in a `my_robot_id/` namespace. The edited code can be found [here](https://github.com/eliabntt/irotate_active_slam/tree/isaac). We still use our `custom_6dof` controller. The behaviour is the same of the case above.

### With joints and pose setpoints

Assuming the joint you want to control is at path `/World/robot/.../PrismaticJoint`

To control the robot within isaac sim you can launch an interactive session and use the following to control joint speeds

```python
import omni.kit.commands
from pxr import Sdf

omni.kit.commands.execute('ChangeProperty',
	prop_path=Sdf.Path('/World/robot/.../PrismaticJoint.drive:linear:physics:targetPosition'),
	value=10,
	prev=prev_value)
```

Or joint positions
```python
import omni.kit.commands
from pxr import Sdf

omni.kit.commands.execute('ChangeProperty',
	prop_path=Sdf.Path('/World/Cone/PrismaticJoint.drive:linear:physics:targetVelocity'),
	value=19.30000028759241,
	prev=0.0)
```

For the latter, you may need to modify the stiffness of the joint with the following

```python
import omni.kit.commands
from pxr import Sdf

omni.kit.commands.execute('ChangeProperty',
	prop_path=Sdf.Path('/World/Cone/PrismaticJoint.drive:linear:physics:stiffness'),
	value=1000,
	prev=0.0)
```

At this point, you can create custom trajectories easily and run them within the main simulation loop. We did that in `savana_simulation` to control our robot. The joint parameters may require some tuning, especially in the maximum force and the stiffness. I suggest you test with long shot trajectories manually in simulation before generating data.

_NOTE_ you cannot have a separate program running giving these commands since it needs to be bound to the main simulation environment to access the prim itself.
ROS is a simple way to go around this limitation. However, you can have some FSM that publishes setpoints and the main loop that uses the commands above to modify the joints.

### Without joints

You can animate the camera by hand/programmatically, using keypoints such as the one used for the flying objects. Note that this will let the robot pass through any obstacle and won't have physics.

Alternatively, you could use teleporting.