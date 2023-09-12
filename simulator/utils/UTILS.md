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

Edit if you need new sensors, publish different data, or remove sensors. You would also like to clean the code or add noise to the data directly here.

## Simulation utils

Mainly used for configuration settings (enalbe/disable extensions, change raytracing/pathtracing options), check that nucleus is powered up and ros is working, and manage the timeline.
