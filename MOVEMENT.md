# Movement control
## Autonomous exploration (paper version)

Use the simulation code to use our full system. 
The trajectory will be created by FUEL, passed to a NMPC, to our PID controller, and published as joint commands. 
Once the simulation is launched you do not need to do anything at all. 

Once recording starts a topic will be published in the `/starting_experiment` topic. 

## With ROS
Here you have many options:

1. You can use a simple script in python to send commands to /my_robot_0/joint_commands. Those will be executed accordingly to the stiffness and damping parameters. This can be launched with the Isaac's ros python installation as a ros node.
2. You can use a simple script in python to send commands to /cmd_vel and use our PID controller that you can find [here]() or any other controller that you wish.
3. You can load your own robot USD that works with one of the Isaac embedded examples. Some of them works with moveit, some of them work with `/cmd_vel`. Those should work out of the box.
  To load them simply change the `robot_base_prim_path` when calling `import_robot`. You can use local usd files or nucleus's ones. 
4. (custom) Teleop twist keyboard/joy works as long as there is a translation between `/cmd_vel` and `joint_commands` (either from USD's controller or external controller as we do)
5. A trajectory publisher to work with an NMPC. The NMPC can then either publish desired waypoints or velocities. Eitherway, you can use the [controller]() to translate those to joint commands or resort to 2.
6. Write your own FSM that resort to one of the previous options...
7. ...
	
_Note that many of these can also be published by command line (`rostopic pub ...`)_

*Remember to tune the PID accordingly and to eventually adapt to your robot joints in [here]().*

## Movement Without ROS

### With joints

Assuming the joint you want to control is at path `/World/robot/.../PrismaticJoint`

To control the robot within isaac sim you can launch an interactive session and use the following to control joint speeds

```
import omni.kit.commands
from pxr import Sdf

omni.kit.commands.execute('ChangeProperty',
	prop_path=Sdf.Path('/World/robot/.../PrismaticJoint.drive:linear:physics:targetPosition'),
	value=10,
	prev=prev_value)
```

Or joint positions
```
import omni.kit.commands
from pxr import Sdf

omni.kit.commands.execute('ChangeProperty',
	prop_path=Sdf.Path('/World/Cone/PrismaticJoint.drive:linear:physics:targetVelocity'),
	value=19.30000028759241,
	prev=0.0)
```

For the latter, you may need to modify the stiffness of the joint with the following

```
import omni.kit.commands
from pxr import Sdf

omni.kit.commands.execute('ChangeProperty',
	prop_path=Sdf.Path('/World/Cone/PrismaticJoint.drive:linear:physics:stiffness'),
	value=1000,
	prev=0.0)
```

At this point, you can create custom trajectories easily and run them within the main simulation loop.
_NOTE_ you cannot have a separate program running since it needs to be bound to the main simulation environment to access the prim itself.
ROS is a simple way to go around this limitation.integers

** NOTE ** This is more difficult to manage (e.g. you need to keep checking if you are arrived before sending the new setpoint), but will effectively use physics and publish accelerations/velocities.

### Without joints
You can animate the camera by hand/programmatically, using keypoints (see [here]()). Note that this will let the robot pass through any obstacle.
**NOTE: ** This won't publish accelerations/velocities except for the IMU. You can find them afterwards by using the pose log.
