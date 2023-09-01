"""
Use this code to colorize the generated data.

The code is thought to colorize all the data, create videos, and fix the vertical fov issue.
Please check the arguments to understand how to use it.
Please set the corresponding data_enabled to False if you do not want to colorize some kind of data (eg. depth_enabled)
"""

import math
import argparse
import colorsys
import confuse
import copy
import cv2
import ipdb
import numpy as np
import os
import pickle as pkl
import random
from PIL import Image, ImageDraw


def project_pinhole(points, view_proj_matrix):
  """
  Project 3D points to 2D camera view using a pinhole camera model.
  Args:
      points (numpy.ndarray): Array of points in world frame of shape (num_points, 3).
      viewport (omni.kit.viewport._viewport.IViewportWindow): Viewport from which to retrieve/create sensor.

  Returns:
      (numpy.ndarray): Image-space points of shape (num_points, 3)
  """
  homo = np.pad(points, ((0, 0), (0, 1)), constant_values=1.0)
  tf_points = np.dot(homo, view_proj_matrix)
  tf_points = tf_points / (tf_points[..., -1:])
  tf_points[..., :2] = 0.5 * (tf_points[..., :2] + 1)
  return tf_points[..., :3]


def random_colours(N, enable_random=True, num_channels=3):
  """
  Generate random colors.
  Generate visually distinct colours by linearly spacing the hue
  channel in HSV space and then convert to RGB space.
  """
  start = 0
  if enable_random:
    random.seed(10)
    start = random.random()
  hues = [(start + i / N) % 1.0 for i in range(N)]
  colours = [list(colorsys.hsv_to_rgb(h, 0.9, 1.0)) for i, h in enumerate(hues)]
  if num_channels == 4:
    for color in colours:
      color.append(1.0)
  if enable_random:
    random.shuffle(colours)
  return colours


def colorize_bboxes(bboxes_2d_data, rgb, num_channels=4):
  """ Colorizes 2D bounding box data for visualization.


      Args:
          bboxes_2d_data (numpy.ndarray): 2D bounding box data from the sensor.
          rgb (numpy.ndarray): RGB data from the sensor to embed bounding box.
          num_channels (int): Specify number of channels i.e. 3 or 4.
  """
  obj_name_list = []
  rgb_img = Image.fromarray(rgb).convert("RGBA")
  rgb_img2 = Image.fromarray(rgb)
  overlay = Image.new("RGBA", rgb_img.size, (0, 0, 0, 0))
  rgb_img_draw = ImageDraw.Draw(overlay)
  rgb_img_draw2 = ImageDraw.Draw(rgb_img2)

  for bbox_2d in bboxes_2d_data:
    obj_name_list.append(bbox_2d[1])

  obj_name_list_np = np.unique(np.array(obj_name_list))
  color_list = random_colours(len(obj_name_list_np.tolist()), True, num_channels)

  for bbox_2d in bboxes_2d_data:
    index = np.where(obj_name_list_np == bbox_2d[1])[0][0]
    bbox_color = color_list[index]
    outline = (int(255 * bbox_color[0]), int(255 * bbox_color[1]), int(255 * bbox_color[2]))
    if num_channels == 4:
      outline = (
        int(255 * bbox_color[0]),
        int(255 * bbox_color[1]),
        int(255 * bbox_color[2]),
        int(255 * bbox_color[3]),
      )
    fill = (
      int(255 * bbox_color[0]),
      int(255 * bbox_color[1]),
      int(255 * bbox_color[2]),
      int(0.25 * 255),
    )
    rgb_img_draw.rectangle([(bbox_2d[6], bbox_2d[7]), (bbox_2d[8], bbox_2d[9])], fill=fill, outline=outline, width=3)
    rgb_img_draw2.rectangle([(bbox_2d[6], bbox_2d[7]), (bbox_2d[8], bbox_2d[9])], outline=outline, width=3)

    bboxes_2d_rgb = Image.alpha_composite(rgb_img, overlay)
    bboxes_2d_rgb = np.array(bboxes_2d_rgb)
    bboxes_2d_rgb2 = np.array(rgb_img2)
  bboxes_2d_rgb3 = np.array(Image.alpha_composite(rgb_img2.convert("RGBA"), overlay))
  return bboxes_2d_rgb3  # , bboxes_2d_rgb2 #only boxes


def colorize_depth(depth_image):
  """
  It takes a depth image, normalizes it, and then maps it to a color image

  :param depth_image: The depth image to be colorized
  :return: The colorized depth image.
  """
  height, width = depth_image.shape[:2]
  colorized_image = np.zeros((height, width, 4))
  depth_image *= 100
  depth_image = np.reciprocal(depth_image)
  depth_image[depth_image == 0.0] = 1e-5
  depth_image = np.clip(depth_image, 0, 255)
  depth_image -= np.min(depth_image)
  if np.max(depth_image) > 0:
    depth_image /= np.max(depth_image) + 1e-8
  colorized_image[:, :, 0] = depth_image
  colorized_image[:, :, 1] = depth_image
  colorized_image[:, :, 2] = depth_image
  colorized_image[:, :, 3] = 1
  colorized_image = (colorized_image * 255).astype(np.uint8)
  return colorized_image


def colorize_semantic_from_instance(instance_image, instance_mappings, sem = False):
  """
  It takes the instance image and the instance mappings and returns a colorized image

  :param instance_image: the instance image from the instance segmentation
  :param instance_mappings: a list of dictionaries, each of which has the following keys:
  """
  if len(instance_mappings) == 0:
    segmentation_image = np.zeros_like(instance_image)
    segmentation_ids = np.unique(segmentation_image)
    num_colours = len(segmentation_ids)

    # This is to avoid generating lots of colours for semantic classes not in frame
    lut = np.array([segmentation_ids, list(range(num_colours))])
    re_instanced = lut[1, np.searchsorted(lut[0, :], segmentation_image)]

    colours = np.array([[0.0] * 4] + random_colours(num_colours))
  else:
    semantic_instances = {}
    changed = np.zeros(instance_image.shape)
    for im in instance_mappings[::-1]:
      semantic_instances.setdefault(im["semanticId"], []).extend(im["instanceIds"])
      changed[instance_image == im["uniqueId"]] = max(im["instanceIds"])
    instance_image = changed.astype(np.uint32)
    max_semantic_instance_id = np.max([max(il) for _, il in semantic_instances.items()])
    max_instance_id = instance_image.max()
    lut = np.zeros(max(max_semantic_instance_id, max_instance_id) + 1, dtype=np.uint32)
    if sem:
      for i, (_, il) in enumerate(semantic_instances.items()):
        lut[np.array(il)] = i + 1  # +1 to differentiate from background
      re_instanced = np.take(lut, instance_image)
      colours = np.array([[0.0] * 3] + random_colours(len(semantic_instances)))
    else:
      colours = np.array([[0.0] * 3] + random_colours(len(lut)))
      re_instanced = instance_image

  rgb = np.zeros((re_instanced.shape[0], re_instanced.shape[1], 3))
  for i in range(len(colours)):
    rgb[re_instanced == i] = colours[i]
  rgb = rgb * 255
  return rgb.astype(np.uint8)


def colorize_bboxes_3d(bboxes_3d_corners, rgb):
  """
  > It takes a list of 3D bounding boxes and a RGB image, and returns the RGB image with the 3D bounding boxes drawn on it

  :param bboxes_3d_corners: in the local camera frame
  :param rgb: the image
  :return: the image with the bounding boxes drawn on it.
  """
  height, width = rgb.shape[:2]

  # FILTER BOXES
  mask_uv = ~np.any(np.all(bboxes_3d_corners < 0, axis=1), axis=1) & ~np.any(
    np.all(bboxes_3d_corners > 1, axis=1), axis=1
  )
  mask_z = np.all(np.all(bboxes_3d_corners[..., 2:] >= 0, axis=1), axis=1) & np.all(
    np.all(bboxes_3d_corners[..., 2:] <= 1, axis=1), axis=1
  )
  bboxes_3d_corners = bboxes_3d_corners[mask_uv & mask_z]

  bboxes_3d_corners = bboxes_3d_corners[..., :2].reshape(-1, 8, 2) * np.array([[width, height]])
  face_idx_list = [[0, 1, 3, 2], [4, 5, 7, 6], [2, 3, 7, 6], [0, 1, 5, 4], [0, 2, 6, 4], [1, 3, 7, 5]]
  colours = random_colours(len(face_idx_list))

  master_overlay_img = Image.new("RGBA", (width, height), (0, 0, 0, 0))

  for face_idxs, colour in zip(face_idx_list, colours):
    overlay = Image.new("RGBA", (width, height))
    draw = ImageDraw.Draw(overlay)
    colour = [int(c * 255) for c in colour]
    for p in bboxes_3d_corners:
      draw.polygon([tuple(xy) for xy in p[face_idxs]], fill=tuple([*colour[:3], 120]))
      draw.line([tuple(xy) for xy in p[face_idxs]], width=3, fill=tuple(colour))

    master_overlay_img = Image.alpha_composite(master_overlay_img, overlay)

  rgb_img = Image.fromarray(rgb).convert("RGBA")
  rgb_img = Image.alpha_composite(rgb_img, master_overlay_img)
  return np.asarray(rgb_img)


def colorize_normals(normals):
  """
  It takes a 3-channel array of normals, and returns a 4-channel array of normals with the background pixels set to
  transparent

  :param normals: a numpy array of shape (H, W, 3) containing the surface normals
  :return: the normals of the image.
  """
  background_mask = np.sum(normals, axis=-1) == 0.0
  # normalize from [-1, 1] to [0, 255]
  normals = (normals + 1.0) / 2 * 255
  # Set background alpha to 0.
  normals = np.pad(normals, ((0, 0), (0, 0), (0, 1)), constant_values=255)
  normals[background_mask, 3] = 0.
  return normals.astype(np.uint8)


def colorize_motion_vector(data):
  """Convert motion vector into colored image. The conversion is done by mapping
  3D direction vector to HLS space, then converted to RGB.
  Args:
      data (numpy.array): data returned by the annotator of shape (H, W, 4).
  Return:
      (np.array): Data converted to uint8 RGBA image.
  """
  r, theta, phi = _cartesian_to_spherical(data[:, :, :3])
  phi += np.pi
  theta_degree = theta * 180 / np.pi
  phi_degree = phi * 180 / np.pi
  h = phi_degree / 360

  l = theta_degree / 180
  r = cv2.normalize(r, None, 0, 1, cv2.NORM_MINMAX)
  pixels = np.dstack((h * 180, l * 255, r * 255)).astype(np.uint8)
  rgb = cv2.cvtColor(pixels, cv2.COLOR_HLS2RGB)

  return rgb


def _cartesian_to_spherical(xyz):
  """
  It takes a 3D Cartesian coordinate and returns the corresponding spherical coordinates

  :param xyz: the 3D coordinates of the points in the image
  """
  h, w = xyz.shape[0], xyz.shape[1]
  xyz = xyz.reshape(-1, 3)
  xy = xyz[:, 0] ** 2 + xyz[:, 1] ** 2
  r = np.sqrt(xy + xyz[:, 2] ** 2)
  theta = np.arctan2(np.sqrt(xy), xyz[:, 2])  # for elevation angle defined from Z-axis down
  phi = np.arctan2(xyz[:, 1], xyz[:, 0])  # for elevation angle defined from XY-plane up
  return r.reshape(h, w), theta.reshape(h, w), phi.reshape(h, w)


def boolean_string(s):
  """
  It takes a string and returns a boolean

  :param s: the string to convert
  :return: The boolean value of the string.
  """
  if s.lower() not in {'false', 'true'}:
    raise ValueError('Not a valid boolean string')
  return s.lower() == 'true'


parser = argparse.ArgumentParser(description="Colorize data")
parser.add_argument("--viewport_folder", type=str)
parser.add_argument("--img_id", type=str, default="-1")
parser.add_argument("--save_imgs", type=boolean_string, default=True)
parser.add_argument("--save_video", type=boolean_string, default=False)
parser.add_argument("--always_update_map", type=boolean_string, default=False)
parser.add_argument("--semantics", type=boolean_string, default=False)
parser.add_argument("--convert_depth", type=boolean_string, default=True) # used to better visualize inverse depth
parser.add_argument("--corrected_bbox_folder", type=str, default="")
parser.add_argument("--vertical_aperture", type=float, default=2.32)
parser.add_argument("--change_aperture", type=boolean_string, default=False)
parser.add_argument("--output_dir", type=str)
args, unknown = parser.parse_known_args()

config = confuse.Configuration("ColorizeData", __name__)
config.set_args(args)

minid = 1
maxid = 1801
isdigit = False
try:
  int(config["img_id"].get())
  isdigit = True
except:
  isdigit = False

if isdigit:
  img_id = int(config["img_id"].get())
  if img_id <= -1:
    print("Processing all images")
  else:
    minid = img_id
    maxid = img_id + 1
  ids = [i for i in range(minid, maxid)]
else:
  ids = [config["img_id"].get()]

vertical_aperture = config["vertical_aperture"].get()
change_aperture = config["change_aperture"].get()
viewport = config["viewport_folder"].get()
subfolders = os.listdir(config["viewport_folder"].get())
depth_enabled = "depth" in subfolders
depthLinear_enabled = "depthLinear" in subfolders
normals_enabled = "normals" in subfolders
bbox2d_enabled = "bbox_2d_tight" in subfolders
bbox3d_enabled = "bbox_3d" in subfolders  # todo these need to be fixed
instance_enabled = "instance" in subfolders
sem_enabled = "instance" in subfolders and config["semantics"].get()
motion_enabled = "motion-vector" in subfolders
always_update_map = config["always_update_map"].get()

save_video = config["save_video"].get()
save_img = config["save_imgs"].get()
if save_video or save_img:
  outdir = config["output_dir"].get()
  if not os.path.exists(config["output_dir"].get()):
    os.makedirs(outdir)

if config["corrected_bbox_folder"].get() != "":
  corrected_bbox_folder = config["corrected_bbox_folder"].get()
else:
  corrected_bbox_folder = None

old_instance_map = None
vrgb, vdepth, vdepthLinear, vnormals, vbbox2d, vbbox3d, vinstance, vmotion, vsem = [], [], [], [], [], [], [], [], []
for i in ids:
  rgb = cv2.imread(os.path.join(viewport, "rgb", f"{i}.png"))
  if save_img:
    cv2.imwrite(os.path.join(outdir, f"rgb_{i}.png"), rgb)
  if save_video:
    vrgb.append(os.path.join(outdir, f"rgb_{i}.png"))

  if depthLinear_enabled:
    depth = np.load(os.path.join(viewport, "depthLinear", f"{i}.npy"))
    depth = colorize_depth(depth)
    if save_img:
      cv2.imwrite(os.path.join(outdir, f"depthLinear_{i}.png"), depth)
    if save_video:
      vdepthLinear.append(os.path.join(outdir, f"depthLinear_{i}.png"))

  if depth_enabled:
    depth = np.load(os.path.join(viewport, "depth", f"{i}.npy"))
    if config["convert_depth"].get():
      depth = 1/depth
    depth = colorize_depth(depth)
    if save_img:
      cv2.imwrite(os.path.join(outdir, f"depth_{i}.png"), depth)
    if save_video:
      vdepth.append(os.path.join(outdir, f"depth_{i}.png"))

  if normals_enabled:
    normals = np.load(os.path.join(viewport, "normals", f"{i}.npy"))
    normals = colorize_normals(normals)

    if save_img:
      cv2.imwrite(os.path.join(outdir, f"normals_{i}.png"), normals)
    if save_video:
      vnormals.append(os.path.join(outdir, f"normals_{i}.png"))

  if bbox2d_enabled:
    bbox2d = np.load(os.path.join(viewport, "bbox_2d_tight", f"{i}.npy"), allow_pickle=True)
    rgb_data = copy.deepcopy(rgb)
    bbox2d = colorize_bboxes(bbox2d, rgb_data)

    if save_img:
      cv2.imwrite(os.path.join(outdir, f"bbox2d_{i}.png"), bbox2d)
    if save_video:
      vbbox2d.append(os.path.join(outdir, f"bbox2d_{i}.png"))

  if bbox3d_enabled:
    bbox3d = np.load(os.path.join(viewport, "bbox_3d", f"{i}.npy"), allow_pickle=True)
    viewport_mat = np.load(os.path.join(viewport, "camera", f"{i}.npy"), allow_pickle=True)
    view_mat = viewport_mat.item()["view_projection_matrix"]

    pose_mat = viewport_mat.item()["pose"]
    if change_aperture:
      viewproj_mat = np.dot(pose_mat, view_mat)
      vertical_aperture = vertical_aperture
      vfov = 2 * math.atan(vertical_aperture / (2 * viewport_mat.item()["focal_length"]))
      viewproj_mat[1,1] = 1 / math.tan(vfov / 2)
      viewproj_mat = np.dot(np.linalg.inv(pose_mat), viewproj_mat)
    corners = project_pinhole(bbox3d["corners"].reshape(-1, 3), viewproj_mat)
    corners = corners.reshape(-1, 8, 3)
    rgb_data = copy.deepcopy(rgb)
    e = []
    for idx,bb in enumerate(bbox3d):
      if bb['semanticLabel'] in ['zebra','human','google','shapenet']:
        e.append(corners[idx])

    if corrected_bbox_folder is not None:
      corrected_bbox = np.load(os.path.join(corrected_bbox_folder, f"{i}.npy"), allow_pickle=True)
      corrected_bbox = corrected_bbox.item()
      for idx, bb in enumerate(bbox3d):
        if bb[1] in corrected_bbox['bbox3d']:
          print(f"Correcting bbox3d for {bb[1]}")
          # if corrected_bbox['bbox3d'] is dictionary
          if isinstance(corrected_bbox['bbox3d'][bb[1]], dict):
            bbox3d[idx]["corners"] = corrected_bbox['bbox3d'][bb[1]]["oriented"] / 0.01
          else:
            bbox3d[idx]["corners"] = corrected_bbox['bbox3d'][bb[1]] / 0.01

    bbox3d = colorize_bboxes_3d(np.array(e), rgb_data)

    if save_img:
      cv2.imwrite(os.path.join(outdir, f"bbox3d_{i}.png"), bbox3d)
    if save_video:
      vbbox3d.append(os.path.join(outdir, f"bbox3d_{i}.png"))

  if instance_enabled:
    instance = np.load(os.path.join(viewport, "instance", f"{i}.npy"), allow_pickle=True)
    if old_instance_map is None or always_update_map:
      old_instance_map = copy.deepcopy(instance[1])
    instance[1] = copy.deepcopy(old_instance_map)
    instance_img = colorize_semantic_from_instance(instance[0], instance[1])

    if save_img:
      cv2.imwrite(os.path.join(outdir, f"instance_{i}.png"), instance_img)
    if save_video:
      vinstance.append(os.path.join(outdir, f"instance_{i}.png"))

      if sem_enabled:
        sem = colorize_semantic_from_instance(instance[0], instance[1], sem=True)
        if save_img:
          cv2.imwrite(os.path.join(outdir, f"sem_{i}.png"), sem)
        if save_video:
          vsem.append(os.path.join(outdir, f"sem_{i}.png"))

  if motion_enabled:
    motion = np.load(os.path.join(viewport, "motion-vector", f"{i}.npy"), allow_pickle=True)
    motion = colorize_motion_vector(motion)

    if save_img:
      cv2.imwrite(os.path.join(outdir, f"motion_{i}.png"), motion)
    if save_video:
      vmotion.append(os.path.join(outdir, f"motion_{i}.png"))

if save_video:
  height, width, layers = rgb.shape
  for v in zip([vrgb, vdepth, vdepthLinear, vnormals, vbbox2d, vbbox3d, vinstance, vmotion, vsem],
               ["rgb", "depth", "depthLinear", "normals", "bbox2d", "bbox3d", "instance", "motion", "sem"]):
    if len(v[0]) > 0:
      video = cv2.VideoWriter(os.path.join(outdir, f"{v[1]}.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 30, (width, height))
      for img_path in v[0]:
        img = cv2.imread(img_path)
        if img.shape[2] < 3:
          img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        video.write(img[:, :, :3])
      video.release()
      os.system("ffmpeg -i " + os.path.join(outdir, f"{v[1]}.mp4") + " -vcodec libx264 -y " + os.path.join(outdir,
                                                                                                           f"{v[1]}_conv.mp4"))
