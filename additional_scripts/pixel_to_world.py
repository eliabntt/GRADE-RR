"""
This code serve as an example to project the points from the pixel coordinates to the world coordinates.
You need the camera pose and projection matrix, as well as clearly the pixel depth.

Those are available in the viewport folder, for example:
Viewport0/camera
Viewport0/depth (or depthLinear)

You will load the camera viewport_mat from the camera folder.
This dictionary will have the view projection matrix and the global camera pose
They use a near/far clipping plane model, and not a focal length model.

At the end of the file you can also check how to use the focal length model, but you need to know the focal length of the camera
"""
viewport_mat = np.load(os.path.join(viewport, 'camera',f'{i}.npy'), allow_pickle=True)
# in Isaac view_projection is np.dot(view_matrix, proj_matrix)
# view_matrix is local to world, i.e. the inverse of the pose matrix
# the proj_matrix use the near far clipping plane model
# a = -1.0 / np.tan(np.radians(fov / 2))
# b = -a * aspect_ratio
# c = z_far / (z_far - z_near)
# d = z_near * z_far / (z_far - z_near)
# Construct the camera projection matrix
# projection_matrix = np.array([
#    [a, 0.0, 0.0, 0.0],
#    [0.0, b, 0.0, 0.0],
#    [0.0, 0.0, c, 1.0],
#    [0.0, 0.0, d, 0.0]
# ])
view_mat = viewport_mat.item()["view_projection_matrix"]
pose_mat = viewport_mat.item()["pose"]
inv_VP = np.linalg.inv(view_mat)

pixel_x = ....
pixel_y = ....
pixel_d = ....

width = viewport_mat['resolution']['width']
width = viewport_mat['resolution']['height']
F = viewport_mat['clipping_range'][1]
N = viewport_mat['clipping_range'][0]
W = -pixel_d

ndc_x = (2 * pixel_x) / width - 1
ndc_y = 1 - (2 * pixel_y) / height
Z = ( (W*F/(F-N)) + N*F/(F-N) )/(W)
xyz = np.array([ndc_x, ndc_y, Z, 1]) * W
xyz = np.dot(xyz, inv_VP)

# alternatively consider that a = -fx, b = fy, cx = widht / 2, cy = height /2
# and that the pose_mat has the translation in the last ROW (in unit coordinates, so mind the scale)
tmp = np.dot(pose_mat, view_mat)
fx = -tmp[0,0]
fy = tmp[1,1]
cx = width / 2
cy = height / 2

x = (px - cx) * d / fx
y = (py - cy) * d / fy
pt = [x,y,z,1]
xyz = np.dot(cpose.T, pt)[:3]
