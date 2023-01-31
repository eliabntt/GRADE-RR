To strip completely ros from the main program is possible.
We plan to release an example script in the future.

If you do not wish to use our placement strategy or our control just comment out the [roslauncher]() setup.

If you do not wish to use any ros at all our suggestion is to do the above and keep everything else as-is.

This means installing ros and eventually do the following changes:
- remove the smaller camera from the code [here]() and remove the `/robot/camera_link/Camera` from the USD file (important otherwise the viewport will be created anyway)
- change the extension [here]() to save all the images and not just one every two viewports
- change [here]() and remove this camera (eventually add this camera to the ros camera list)
- if you want save this data in additional arrays:
	- the [imu]() should be trivial
	- the [odom]() is already returned by the function itself
